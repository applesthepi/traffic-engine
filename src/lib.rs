#![feature(get_mut_unchecked)]
#![feature(let_chains)]

#[macro_use]
pub mod network;
#[macro_use]
pub mod object;

use std::{sync::{atomic::Ordering, Arc}, thread, time::Duration};

use glam::Vec2;
use network::{Network, vehicle::Vehicle, lane::LaneIdentity};

use crate::network::{clip::Clip, band::Band, lane::Lane};

// #[tokio::main]
// async fn main() {
// 	let mut network: Network = Network::default();
// 	setup(&mut network);
// }


#[allow(unused_variables)]
pub async fn setup(
	network: &Arc<Network>
) {
	network.allocation.staged_vehicle_batch.read().await.write().await.id = 1;
	network.allocation.vehicle_batch_counter.store(1, Ordering::SeqCst);
	let spread: f32 = 150.0;

	// {
	// 	let allocation = network.allocation.clone();
	// 	let mut wa_vb = allocation.vehicle_batches.write().await;
	// 	let new_id = network.vehicle_batch_counter.fetch_add(1, Ordering::SeqCst) + 1;
	// 	wa_vb.insert(new_id, allocation.staged_vehicle_batch.clone());
	// }

	let clip_a = Clip::new(network).await;
	let clip_b = Clip::new(network).await;
	let clip_c = Clip::new(network).await;
	let clip_d = Clip::new(network).await;
	let clip_e = Clip::new(network).await;
	let clip_f = Clip::new(network).await;
	let clip_g = Clip::new(network).await;
	let clip_h = Clip::new(network).await;
	let clip_i = Clip::new(network).await;
	let clip_j = Clip::new(network).await;
	
	let band_a = Band::new(network, clip_a, clip_b).await;
	let band_b = Band::new(network, clip_b, clip_c).await;
	let band_c = Band::new(network, clip_c, clip_d).await;
	let band_d = Band::new(network, clip_b, clip_d).await;
	let band_e = Band::new(network, clip_d, clip_e).await;
	let band_f = Band::new(network, clip_e, clip_f).await;
	let band_g = Band::new(network, clip_d, clip_g).await;
	let band_h = Band::new(network, clip_g, clip_h).await;
	let band_i = Band::new(network, clip_f, clip_i).await;
	let band_j = Band::new(network, clip_i, clip_j).await;
	
	// FIRST STREIGHT

	let lane_a = Lane::from_streight(
		network,
		Vec2::new(spread * 0.0, spread * 0.0),
		Vec2::new(spread * 0.0, spread * 1.0),
		clip_a, clip_b, 0, 0, band_a
	).await;
	let lane_b = Lane::from_streight(
		network,
		Vec2::new(spread * 0.2, spread * 0.0),
		Vec2::new(spread * 0.2, spread * 1.0),
		clip_a, clip_b, 1, 1, band_a
	).await;
	let lane_c = Lane::from_streight(
		network,
		Vec2::new(spread * 0.0, spread * 1.0),
		Vec2::new(spread * 0.0, spread * 2.0),
		clip_b, clip_c, 0, 0, band_b
	).await;

	// SQUIGLE

	let lane_dg = Lane::new(
		network,
		Vec2::new(spread * 0.2, spread * 1.0),
		Vec2::new(spread * 0.2, spread * 2.2),
		Vec2::new(spread * 0.4, spread * 1.8),
		Vec2::new(spread * 0.4, spread * 3.0),
		clip_b, clip_d, 1, 2, band_d
	).await;

	// EXPAND

	let lane_e = Lane::from_streight(
		network,
		Vec2::new(spread * 0.0, spread * 2.0),
		Vec2::new(spread * 0.0, spread * 3.0),
		clip_c, clip_d, 0, 0, band_c
	).await;
	let lane_f = Lane::from_streight(
		network,
		Vec2::new(spread * 0.0, spread * 2.0),
		Vec2::new(spread * 0.2, spread * 3.0),
		clip_c, clip_d, 0, 1, band_c
	).await;

	// STREIGHT AWAY

	let lane_i = Lane::from_streight(
		network,
		Vec2::new(spread * 0.0, spread * 3.0),
		Vec2::new(spread * 0.0, spread * 4.0),
		clip_d, clip_e, 0, 0, band_e
	).await;
	let lane_j = Lane::from_streight(
		network,
		Vec2::new(spread * 0.2, spread * 3.0),
		Vec2::new(spread * 0.2, spread * 4.0),
		clip_d, clip_e, 1, 1, band_e
	).await;
	let lane_k = Lane::from_streight(
		network,
		Vec2::new(spread * 0.4, spread * 3.0),
		Vec2::new(spread * 0.4, spread * 4.0),
		clip_d, clip_e, 2, 2, band_e
	).await;

	// MERGE A

	let lane_m = Lane::from_streight(
		network,
		Vec2::new(spread * 0.0, spread * 4.0),
		Vec2::new(spread * 0.2, spread * 5.0),
		clip_e, clip_f, 0, 0, band_f
	).await;
	let lane_n = Lane::from_streight(
		network,
		Vec2::new(spread * 0.2, spread * 4.0),
		Vec2::new(spread * 0.2, spread * 5.0),
		clip_e, clip_f, 1, 0, band_f
	).await;
	let lane_o = Lane::from_streight(
		network,
		Vec2::new(spread * 0.4, spread * 4.0),
		Vec2::new(spread * 0.4, spread * 5.0),
		clip_e, clip_f, 2, 1, band_f
	).await;

	// MERGE INT

	let lane_p = Lane::from_streight(
		network,
		Vec2::new(spread * 0.2, spread * 5.0),
		Vec2::new(spread * 0.2, spread * 6.0),
		clip_f, clip_i, 0, 0, band_i
	).await;
	let lane_q = Lane::from_streight(
		network,
		Vec2::new(spread * 0.4, spread * 5.0),
		Vec2::new(spread * 0.4, spread * 6.0),
		clip_f, clip_i, 1, 1, band_i
	).await;

	// MERGE B

	let lane_r = Lane::from_streight(
		network,
		Vec2::new(spread * 0.2, spread * 6.0),
		Vec2::new(spread * 0.2, spread * 7.0),
		clip_i, clip_j, 0, 0, band_j
	).await;
	let lane_s = Lane::from_streight(
		network,
		Vec2::new(spread * 0.4, spread * 6.0),
		Vec2::new(spread * 0.2, spread * 7.0),
		clip_i, clip_j, 1, 0, band_j
	).await;

	// EXIT

	let lane_h = Lane::new(
		network,
		Vec2::new(spread * 0.0, spread * 3.0),
		Vec2::new(spread * 0.0, spread * 3.2),
		Vec2::new(spread * 0.0, spread * 3.7),
		Vec2::new(spread * -0.4, spread * 4.0),
		clip_d, clip_g, 0, 0, band_g
	).await;

	// VEHICLES

	// println!("spawning vehicle on {} {} {} nav to {} {} {}", clip_a, band_a, lane_a, lane_r, band_j, clip_i);
	// let vehicle_1 = Vehicle::new(
	// 	network,
	// 	LaneIdentity {
	// 		lane: lane_a,
	// 		band: band_a,
	// 		clip: clip_a
	// 	},
	// 	LaneIdentity {
	// 		lane: lane_i,
	// 		band: band_e,
	// 		clip: clip_d
	// 	}
	// ).await;

	let c_network = network.clone();
	tokio::spawn(async move {
		thread::sleep(Duration::from_millis(500));
		loop {
			// thread::sleep(Duration::from_millis(3_000));
			Vehicle::new(
				&c_network,
				LaneIdentity {
					lane: lane_a,
					band: band_a,
					clip: clip_a
				},
				LaneIdentity {
					lane: lane_i,
					band: band_e,
					clip: clip_d
				}
			).await;
			thread::sleep(Duration::from_millis(3_000));
			Vehicle::new(
				&c_network,
				LaneIdentity {
					lane: lane_a,
					band: band_a,
					clip: clip_a
				},
				LaneIdentity {
					lane: lane_i,
					band: band_e,
					clip: clip_d
				}
			).await;
			return;
			// println!("{:?}", c_network.allocation.staged_vehicle_batch.read().await);
			// println!("1");
			// return;
		}
	});


	// unsafe {
	// 	spawn_vehicle(&mut network.network, &mut commands, &mut meshes, &mut materials,
	// 		clip_a, band_a, lane_a,
	// 		clip_i, band_j, lane_r
	// 	);
	// }

	// Ok(())
}
