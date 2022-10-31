extern crate nalgebra_glm as glm;

#[macro_use]
pub mod network;

use glm::Vec2;
use network::Network;

use crate::network::{clip::Clip, band::Band, lane::Lane};

// #[tokio::main]
// async fn main() {
// 	let mut network: Network = Network::default();
// 	setup(&mut network);
// }

pub fn setup(
	network: &mut Network
) {
	let spread: f32 = 5.0;

	let clip_a = Clip::new(network);
	let clip_b = Clip::new(network);
	let clip_c = Clip::new(network);
	let clip_d = Clip::new(network);
	let clip_e = Clip::new(network);
	let clip_f = Clip::new(network);
	let clip_g = Clip::new(network);
	let clip_h = Clip::new(network);
	let clip_i = Clip::new(network);
	let clip_j = Clip::new(network);
	
	let band_a = Band::new(network, clip_a, clip_b);
	let band_b = Band::new(network, clip_b, clip_c);
	let band_c = Band::new(network, clip_c, clip_d);
	let band_d = Band::new(network, clip_b, clip_d);
	let band_e = Band::new(network, clip_d, clip_e);
	let band_f = Band::new(network, clip_e, clip_f);
	let band_g = Band::new(network, clip_d, clip_g);
	let band_h = Band::new(network, clip_g, clip_h);
	let band_i = Band::new(network, clip_f, clip_i);
	let band_j = Band::new(network, clip_i, clip_j);
	
	// FIRST STREIGHT

	let lane_a = Lane::from_streight(
		network,
		Vec2::new(0.0, spread * 0.0),
		Vec2::new(0.0, spread * 1.0),
		clip_a, clip_b, 0, 0, band_a
	);
	let lane_b = Lane::from_streight(
		network,
		Vec2::new(3.0, spread * 0.0),
		Vec2::new(3.0, spread * 1.0),
		clip_a, clip_b, 1, 1, band_a
	);
	let lane_c = Lane::from_streight(
		network,
		Vec2::new(0.0, spread * 1.0),
		Vec2::new(0.0, spread * 2.0),
		clip_b, clip_c, 0, 0, band_b
	);

	// SQUIGLE

	let lane_dg = Lane::new(
		network,
		Vec2::new(3.0, spread * 1.0),
		Vec2::new(3.0, spread * 2.2),
		Vec2::new(6.0, spread * 1.8),
		Vec2::new(6.0, spread * 3.0),
		clip_b, clip_d, 1, 2, band_d
	);

	// EXPAND

	let lane_e = Lane::from_streight(
		network,
		Vec2::new(0.0, spread * 2.0),
		Vec2::new(0.0, spread * 3.0),
		clip_c, clip_d, 0, 0, band_c
	);
	let lane_f = Lane::from_streight(
		network,
		Vec2::new(0.0, spread * 2.0),
		Vec2::new(3.0, spread * 3.0),
		clip_c, clip_d, 0, 1, band_c
	);

	// STREIGHT AWAY

	let lane_i = Lane::from_streight(
		network,
		Vec2::new(0.0, spread * 3.0),
		Vec2::new(0.0, spread * 4.0),
		clip_d, clip_e, 0, 0, band_e
	);
	let lane_j = Lane::from_streight(
		network,
		Vec2::new(3.0, spread * 3.0),
		Vec2::new(3.0, spread * 4.0),
		clip_d, clip_e, 1, 1, band_e
	);
	let lane_k = Lane::from_streight(
		network,
		Vec2::new(6.0, spread * 3.0),
		Vec2::new(6.0, spread * 4.0),
		clip_d, clip_e, 2, 2, band_e
	);

	// MERGE A

	let lane_m = Lane::from_streight(
		network,
		Vec2::new(0.0, spread * 4.0),
		Vec2::new(3.0, spread * 5.0),
		clip_e, clip_f, 0, 0, band_f
	);
	let lane_n = Lane::from_streight(
		network,
		Vec2::new(3.0, spread * 4.0),
		Vec2::new(3.0, spread * 5.0),
		clip_e, clip_f, 1, 0, band_f
	);
	let lane_o = Lane::from_streight(
		network,
		Vec2::new(6.0, spread * 4.0),
		Vec2::new(6.0, spread * 5.0),
		clip_e, clip_f, 2, 1, band_f
	);

	// MERGE INT

	let lane_p = Lane::from_streight(
		network,
		Vec2::new(3.0, spread * 5.0),
		Vec2::new(3.0, spread * 6.0),
		clip_f, clip_i, 0, 0, band_i
	);
	let lane_q = Lane::from_streight(
		network,
		Vec2::new(6.0, spread * 5.0),
		Vec2::new(6.0, spread * 6.0),
		clip_f, clip_i, 1, 1, band_i
	);

	// MERGE B

	let lane_r = Lane::from_streight(
		network,
		Vec2::new(3.0, spread * 6.0),
		Vec2::new(3.0, spread * 7.0),
		clip_i, clip_j, 0, 0, band_j
	);
	let lane_s = Lane::from_streight(
		network,
		Vec2::new(6.0, spread * 6.0),
		Vec2::new(3.0, spread * 7.0),
		clip_i, clip_j, 1, 0, band_j
	);

	// EXIT

	let lane_h = Lane::new(
		network,
		Vec2::new(0.0, spread * 3.0),
		Vec2::new(0.0, spread * 3.2),
		Vec2::new(0.0, spread * 3.7),
		Vec2::new(-4.0, spread * 4.0),
		clip_d, clip_g, 0, 0, band_g
	);

	// VEHICLES

	// println!("spawning veh {} {} {}", clip_a, band_a, lane_a);

	// unsafe {
	// 	spawn_vehicle(&mut network.network, &mut commands, &mut meshes, &mut materials,
	// 		clip_a, band_a, lane_a,
	// 		clip_i, band_j, lane_r
	// 	);
	// }

	// Ok(())
}
