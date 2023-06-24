use nalgebra::Vector3;

#[derive(Debug, Default, Clone, Copy)]
pub struct Point {
	pub position: Vector3<f32>,
	pub accumulated_distance: f32,
}