// Contains all basic network data structures.

use std::cell::RefCell;
use std::sync::atomic::AtomicU32;
use std::sync::Arc;
use std::collections::HashMap;
use std::sync::atomic::Ordering;

use bytemuck::Pod;
use bytemuck::Zeroable;
use glam::Vec2;
use tokio::sync::RwLock;
use tokio::sync::RwLockReadGuard;
use tokio::sync::RwLockWriteGuard;
use vulkano::buffer::BufferUsage;
use vulkano::buffer::CpuAccessibleBuffer;
use vulkano::device::Device;
use vulkano::impl_vertex;

#[macro_use]
pub mod clip;
#[macro_use]
pub mod band;
#[macro_use]
pub mod lane;
#[macro_use]
pub mod vehicle;
#[macro_use]
pub mod navigation;

use crate::network::clip::*;
use crate::network::band::*;
use crate::network::lane::*;
use crate::network::vehicle::*;
// use crate::network::navigation::*;

pub const BATCH_COUNT: usize = 10;
pub const LANE_MAX_BRANCH: u8 = 5;

#[macro_export]
macro_rules! network_allocation_mut {
	($network:expr) => {
		unsafe { &mut Arc::get_mut_unchecked(&mut $network).allocation }
		// if cfg!(debug_assertions) {
		// 	&mut Arc::get_mut(&mut $network).unwrap().allocation
		// } else {
		// 	unsafe { &mut Arc::get_mut_unchecked(&mut $network).allocation }
		// }
	};
}
pub(crate) use network_allocation_mut;

#[macro_export]
macro_rules! network_allocation {
	($network:expr) => {
		&$network.allocation
	};
}
// pub(crate) use network_allocation;

#[derive(Default)]
pub struct Network {
	pub allocation: NetworkAllocation,
}

// impl Network {
// 	pub unsafe fn get_allocation(self:Arc<Self>) -> &'static mut NetworkAllocation {
// 		#[cfg(debug_assertions)]
// 		&mut (&mut Arc::get_mut_unchecked(&mut self.clone())).allocation
// 		#[cfg(not(debug_assertions))]
// 		&mut (&mut Arc::get_mut(&mut self.clone()).unwrap()).allocation
// 	}
// }

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Pod, Zeroable)]
pub struct NetworkVertex {
	position: [f32; 2],
}
impl_vertex!(NetworkVertex, position);

impl NetworkVertex {
	pub fn square(
		position: [f32; 2],
		hdis: f32,
		index_offset: u32
	) -> ([NetworkVertex; 4], [u32; 6]) {
		let io = index_offset;
		let r = ([
			NetworkVertex { position: [position[0] - hdis, position[1] + hdis] },
			NetworkVertex { position: [position[0] + hdis, position[1] + hdis] },
			NetworkVertex { position: [position[0] + hdis, position[1] - hdis] },
			NetworkVertex { position: [position[0] - hdis, position[1] - hdis] }
		],
		[
			io + 0, io + 1, io + 2,
			io + 2, io + 3, io + 0
		]);
		// if r.0[1].position[1] == r.0[2].position[1] {
		// 	println!("{:?} {:?}", position, r.0);
		// }
		r
	}
}

#[derive(Default)]
pub struct NetworkAllocation {
	pub clips: Arc<RwLock<HashMap<u32, Arc<RwLock<Clip>>>>>,
	pub bands: Arc<RwLock<HashMap<u32, Arc<RwLock<Band>>>>>,
	pub lanes: Arc<RwLock<HashMap<u32, Arc<RwLock<Lane>>>>>,
	pub vehicle_batches: Arc<RwLock<HashMap<u32, Arc<RwLock<VehicleBatch>>>>>,
	pub staged_vehicle_batch: Arc<RwLock<Arc<RwLock<VehicleBatch>>>>,
	pub unused_vehicle_batchs: Arc<RwLock<Vec<Arc<RwLock<VehicleBatch>>>>>,

	pub clip_count: AtomicU32,
	pub band_count: AtomicU32,
	pub lane_count: AtomicU32,
	pub vehicle_count: AtomicU32,
	pub vehicle_batch_counter: AtomicU32,
}

impl NetworkAllocation {
	pub async fn cycle_svb(&self) {
		let mut wa_svb_con = self.staged_vehicle_batch.write().await;
		let svb_id = wa_svb_con.read().await.id;
		let mut wa_vbs = self.vehicle_batches.write().await;
		wa_vbs.insert(svb_id, wa_svb_con.clone());
		drop(wa_vbs);
		let mut wa_uvb = self.unused_vehicle_batchs.write().await;
		if let Some(vb) = wa_uvb.pop() {
			*wa_svb_con = vb.clone();
		} else {
			let new_id = self.vehicle_batch_counter.fetch_add(1, Ordering::SeqCst) + 1;
			*wa_svb_con = Arc::new(RwLock::new(VehicleBatch::new(new_id)));
		}
	}

	pub async fn clip(&self, clip_id: u32) -> Arc<RwLock<Clip>> {
		let allocation_clips = self.clips.read().await;
		let clip_c = allocation_clips.get(&clip_id).expect("invalid clip id").clone();
		clip_c
	}

	pub async fn band(&self, band_id: u32) -> Arc<RwLock<Band>> {
		let allocation_bands = self.bands.read().await;
		let band_c = allocation_bands.get(&band_id).expect("invalid band id").clone();
		band_c
	}

	pub async fn lane(&self, lane_id: u32) -> Arc<RwLock<Lane>> {
		let allocation_lanes = self.lanes.read().await;
		let lane_c = allocation_lanes.get(&lane_id).expect("invalid lane id").clone();
		lane_c
	}

	pub async fn vehicle_batch(&self, vehicle_batch_id: u32) -> Arc<RwLock<VehicleBatch>> {
		let allocation_vehicle_batches = self.vehicle_batches.read().await;
		let vehicle_batch_c = allocation_vehicle_batches.get(&vehicle_batch_id).expect("invalid vehicle batch id").clone();
		vehicle_batch_c
	}

	pub async fn build(&self, device: &Arc<Device>) -> (Arc<CpuAccessibleBuffer<[NetworkVertex]>>, Arc<CpuAccessibleBuffer<[u32]>>) {
		
		// COLLECT NETWORK BUFFERS
		
		let mut vertices: Vec<NetworkVertex> = Vec::new();
		let mut indices: Vec<u32> = Vec::new();
		let mut index_offset: u32 = 0;
		let allocation_lanes = self.lanes.read().await;
		for lane in allocation_lanes.iter() {
			// TODO: Arc get_mut_unchecked REMOVE RwLock
			for point in lane.1.read().await.points.iter() {
				let square = NetworkVertex::square([point.position.x, point.position.y], 5.0, index_offset);
				index_offset += 4;
				vertices.extend_from_slice(&square.0);
				indices.extend_from_slice(&square.1);
			}
		}

		// BUILD NETWORK BUFFERS

		(
			CpuAccessibleBuffer::from_iter(
				device.clone(),
				BufferUsage {
					vertex_buffer: true,
					..BufferUsage::empty()
				},
				false,
				vertices,
			).unwrap(),
			CpuAccessibleBuffer::from_iter(
				device.clone(),
				BufferUsage {
					index_buffer: true,
					..BufferUsage::empty()
				},
				false,
				indices,
			).unwrap()
		)
	}
}