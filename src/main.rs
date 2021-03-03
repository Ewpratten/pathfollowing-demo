use raylib::prelude::*;
extern crate math;
extern crate nalgebra as na;

const POSE_INTERPOLATE_DISTANCE: f32 = 10.0;

/// Defines a pose in a path
struct PathPose {
    /// 2D positions
    pub pose: na::Vector2<f32>,

    /// Used to track weather this pose has been targeted by the follower
    pub already_referenced: bool,
}

impl PathPose {
    fn new(pose: na::Vector2<f32>) -> PathPose {
        PathPose {
            pose: pose,
            already_referenced: false,
        }
    }
    fn set_referenced(&mut self) {
        self.already_referenced = true;
    }
}

fn add_control_point(
    point: na::Vector2<f32>,
    control_points: &mut Vec<na::Vector2<f32>>,
    path_poses: &mut Vec<PathPose>,
) {
    // Keep track of the control point
    control_points.push(point);

    // If there are already poses in the path, we need to in-fill them
    if path_poses.len() > 0 {
        // Reference the last post
        let last_pose = path_poses.last().unwrap().pose;

        // Get the distance between the last known pose and the control point
        let displacement = point - last_pose;
        let normal = displacement / displacement.norm();

        // Determine the number of inner poses needed
        let inner_count =
            math::round::ceil((displacement.norm() / POSE_INTERPOLATE_DISTANCE) as f64, 0) as u8;

        // Append each inner pose to the path
        for i in 0..inner_count {
            // Scale the normal according to the inner segment
            let scaled_normal = normal * (POSE_INTERPOLATE_DISTANCE * i as f32);

            // Offset this new value
            let scaled_normal = scaled_normal + last_pose;

            // Build a path pose
            let path_pose = PathPose::new(scaled_normal);
            path_poses.push(path_pose);
        }
    }

    // Add the control point to the path
    path_poses.push(PathPose::new(point));
}

fn redraw_path(
    drawing_handle: &mut RaylibDrawHandle,
    control_points: &Vec<na::Vector2<f32>>,
    path_poses: &Vec<PathPose>,
) {
    // Iter each pose
    for pose in path_poses {
        // Get the pose components
        let pose_x = *pose.pose.get(0).unwrap();
        let pose_y = *pose.pose.get(1).unwrap();

        // Render a circle at its position
        drawing_handle.draw_circle(pose_x as i32, pose_y as i32, 2.0, Color::BLACK);
    }

    // Iter each control point
    for point in control_points {
        // Get the point components
        let pose_x = *point.get(0).unwrap();
        let pose_y = *point.get(1).unwrap();

        // Render a circle at its position
        drawing_handle.draw_circle_lines(pose_x as i32, pose_y as i32, 8.0, Color::BLUE);
    }
}

fn redraw_followed_path(
    drawing_handle: &mut RaylibDrawHandle,
    path_poses: &mut Vec<PathPose>,
    lookahead_radius: f32,
) {
    // Skip this if there are no poses in the path
    if path_poses.len() == 0 {
        return;
    }

    // Clear all the already_referenced trackers
    for path_pose in path_poses.into_iter() {
        path_pose.already_referenced = false;
    }

    // Create a position for the "turtle" (being the start of the path)
    let mut turtle = *path_poses.first().unwrap().pose;

    // Keep track of the turtle's current goal
    let mut goal_index = 0;

    for _ in 0..10000 {
        // Find the best goal for the turtle
        for (i, path_pose) in path_poses.into_iter().enumerate() {
            // Get the pose distance
            let d_x = path_pose.pose.x - turtle.x;
            let d_y = path_pose.pose.y - turtle.y;
            let distance = d_x.hypot(d_y);

            // This pose must be closer than the lookahead_radius and not already_referenced
            if distance < lookahead_radius && !path_pose.already_referenced {
                // Select the goal
                path_poses[goal_index].already_referenced = true;
                goal_index = i;

                // Draw the goal selection
                drawing_handle.draw_line(
                    turtle.x as i32,
                    turtle.y as i32,
                    path_poses[goal_index].pose.x as i32,
                    path_poses[goal_index].pose.y as i32,
                    Color::GREEN,
                );

                break;
            }
        }

        // Determine the movement vector for the turtle
        let d_x = path_poses[goal_index].pose.x - turtle.x;
        let d_y = path_poses[goal_index].pose.y - turtle.y;
        let movement = na::Vector2::new(d_x, d_y);

        // Move the turtle in the direction of the goal
        turtle.x += movement.x * 0.01;
        turtle.y += movement.y * 0.01;

        // Plot the movement
        drawing_handle.draw_circle(turtle.x as i32, turtle.y as i32, 1.0, Color::PINK);

        // If the goal is the last point, we can just assume we are done
        if path_poses[goal_index].pose == path_poses.last().unwrap().pose {
            break;
        }
    }
}

fn main() {
    let (mut rl, thread) = raylib::init()
        .size(800, 600)
        .title("Path Following Demo")
        .build();

    // Control and path poses
    let mut control_points: Vec<na::Vector2<f32>> = Vec::new();
    let mut path_poses: Vec<PathPose> = Vec::new();

    while !rl.window_should_close() {
        let mut d = rl.begin_drawing(&thread);

        // Canvas clear
        d.clear_background(Color::WHITE);

        // Force re-draw the path
        redraw_followed_path(&mut d, &mut path_poses, 20.0 + POSE_INTERPOLATE_DISTANCE);
        redraw_path(&mut d, &control_points, &path_poses);

        // Check for clicks on screen
        if d.is_mouse_button_pressed(MouseButton::MOUSE_LEFT_BUTTON) {
            // Get the click vector
            let mouse_pos = d.get_mouse_position();

            // Translate to an na::Vector2
            let mouse_pos = na::Vector2::new(mouse_pos.x, mouse_pos.y);

            // Add control point
            add_control_point(mouse_pos, &mut control_points, &mut path_poses);
        }

        // Handle reset
        if d.is_key_pressed(KeyboardKey::KEY_R) {
            control_points.clear();
            path_poses.clear();
        }
    }
}
