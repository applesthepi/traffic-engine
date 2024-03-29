use std::sync::{atomic::Ordering, Arc, RwLock};

use nalgebra::Vector2;


use crate::{network::{navigation::Point, clip::Fixed, LANE_MAX_BRANCH}, network_allocation};

use super::{Network, vehicle::{VehicleData}, signal::Signal};

#[derive(Debug, Default, Clone, Copy)]
pub struct LaneIdentity {
	pub lane: u32,
	pub band: u32,
	pub clip: u32,
}

#[derive(Debug)]
pub struct Lane {
	pub identity: LaneIdentity,
	pub fw_lanes: Vec<LaneIdentity>,
	pub bw_lanes: Vec<LaneIdentity>,

	pub p1: Vector2<f32>,
	pub p2: Vector2<f32>,
	pub p3: Vector2<f32>,
	pub p4: Vector2<f32>,

	pub points: Vec<Point>,
	pub length: f32,

	pub vehicles: Vec<VehicleData>,
	pub signals: Vec<Arc<dyn Signal>>
}

impl Lane {
	pub fn from_streight(
		network: &Arc<Network>,
		p1: Vector2<f32>, p2: Vector2<f32>,
		clip_bw: u32, clip_fw: u32,
		lnum_bw: u8, lnum_fw: u8,
		band: u32
	) -> u32 {
		let control = (p2 - p1) * 0.1;
		Lane::new(
			network,
			p1,
			p1 + control,
			p2 - control,
			p2,
			clip_bw,
			clip_fw,
			lnum_bw,
			lnum_fw,
			band
		)
	}

	pub fn new(
		network: &Arc<Network>,
		p1: Vector2<f32>, p2: Vector2<f32>, p3: Vector2<f32>, p4: Vector2<f32>,
		clip_bw: u32, clip_fw: u32,
		lnum_bw: u8, lnum_fw: u8,
		band: u32
	) -> u32 {

		let network_c = network.clone();
		let allocation = network_allocation!(network_c);

		// ID

		let id = allocation.lane_count.fetch_add(
			1,
			Ordering::Relaxed
		) + 1;
		// println!("aquired lane id... {}", id);

		// GENERATE POSITIONS

		let mut points: Vec<Point> = Vec::new();
		let mut last_point = p1;
		let mut accumulated_distance: f32 = 0.0;
		let count: u8 = 5;
		for i in 0..count {
			let t: f32 = (i as f32) / ((count - 1) as f32);
			let omt: f32 = 1.0 - t;
			let tm1: Vector2<f32> = p1 * omt.powf(3.0);
			let tm2: Vector2<f32> = p2 * omt.powf(2.0) * t * 3.0;
			let tm3: Vector2<f32> = p3 * omt * t.powf(2.0) * 3.0;
			let tm4: Vector2<f32> = p4 * t.powf(3.0);
			let p: Vector2<f32> = tm1 + tm2 + tm3 + tm4;
			let mut dis: f32 = 0.0;
			if i > 0 {
				dis = last_point.metric_distance(&p);
			}
			last_point = p;
			accumulated_distance += dis;
			points.push(Point{ position: Vector2::new(p.x, p.y), accumulated_distance });
		}

		// ALLOCATE

		let identity = LaneIdentity {
			lane: id,
			band,
			clip: clip_bw
		};
		let mut wa_allocation_lanes = allocation.lanes.write().unwrap();
		wa_allocation_lanes.insert(id, Arc::new(
			RwLock::new(Self {
				p1,
				p2,
				p3,
				p4,
				points,
				identity: identity.clone(),
				fw_lanes: Vec::new(),
				bw_lanes: Vec::new(),
				length: accumulated_distance,
				vehicles: Vec::new(),
				signals: Vec::new()
			})
		));
		drop(wa_allocation_lanes);

		// UPDATE CLIP -> LANE & LANE -> LANE

		let c_clip_fw = allocation.clip(clip_fw);
		let mut wa_clip_fw = c_clip_fw.write().unwrap();
		let c_clip_bw = allocation.clip(clip_bw);
		let mut wa_clip_bw = c_clip_bw.write().unwrap();
		{
			let lanes_fixed = &mut wa_clip_bw.lanes_fixed;
			if (lanes_fixed.len() as u8) < (lnum_bw + 1) {
				lanes_fixed.resize(
					(lnum_bw + 1) as usize,
					Fixed::default()
				);
			}
			let lane_fixed = &mut lanes_fixed[lnum_bw as usize];
			if !lane_fixed.fw.contains(&id) {
				if lane_fixed.fw_count >= LANE_MAX_BRANCH {
					panic!("lane max branch has been exceded");
				}
				lane_fixed.fw[lane_fixed.fw_count as usize] = id;
				lane_fixed.fw_count += 1;
			}
			for i in 0..lane_fixed.bw_count {
				let c_lane_bw = allocation.lane(lane_fixed.bw[i as usize]);
				let mut wa_lane_bw = c_lane_bw.write().unwrap();
				wa_lane_bw.fw_lanes.push(identity.clone());
			}
		} {
			// let lanes_fixed = &mut wa_clip_fw.lanes_fixed;
			// if (lanes_fixed.len() as u8) < (lnum_fw + 1) {
			// 	lanes_fixed.resize(
			// 		(lnum_fw + 1) as usize,
			// 		Fixed::default()
			// 	);
			// }
			// lanes_fixed[lnum_fw as usize].bw = id;
			// let fw_lane = lanes_fixed[lnum_fw as usize].fw;
			// if fw_lane > 0 {
			// 	let c_lane_fw = allocation.lane(fw_lane);
			// 	let mut wa_lane_fw = c_lane_fw.write();
			// 	wa_lane_fw.bw_lanes.push(identity.clone());
			// }




			let lanes_fixed = &mut wa_clip_fw.lanes_fixed;
			if (lanes_fixed.len() as u8) < (lnum_fw + 1) {
				lanes_fixed.resize(
					(lnum_fw + 1) as usize,
					Fixed::default()
				);
			}
			let lane_fixed = &mut lanes_fixed[lnum_fw as usize];
			if !lane_fixed.bw.contains(&id) {
				if lane_fixed.bw_count >= LANE_MAX_BRANCH {
					panic!("lane max branch has been exceded");
				}
				lane_fixed.bw[lane_fixed.bw_count as usize] = id;
				lane_fixed.bw_count += 1;
			}
			for i in 0..lane_fixed.fw_count {
				let c_lane_fw = allocation.lane(lane_fixed.fw[i as usize]);
				let mut wa_lane_fw = c_lane_fw.write().unwrap();
				wa_lane_fw.bw_lanes.push(identity.clone());
			}
		}

		// RESIZE BAND

		let c_band = allocation.band(band);
		let mut wa_band = c_band.write().unwrap();
		{
			let band_w = &mut wa_band;
			if band_w.empty {
				band_w.empty = false;
				band_w.src_min = lnum_bw;
				band_w.src_max = lnum_bw;
				band_w.dst_min = lnum_fw;
				band_w.dst_max = lnum_fw;
			} else {
				band_w.src_min = band_w.src_min.min(lnum_bw);
				band_w.src_max = band_w.src_max.max(lnum_bw);
				band_w.dst_min = band_w.src_min.min(lnum_fw);
				band_w.dst_max = band_w.src_max.max(lnum_fw);
			}
		}

		// UPDATE CLIP -> BAND

		let mut clip_bw_w = wa_clip_bw;
		if !clip_bw_w.fw_bands.contains(&band) {
			clip_bw_w.fw_bands.push(band);
		}

		// println!("...aquired lane id {}", id);

		id
	}
}