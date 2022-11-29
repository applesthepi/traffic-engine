// Contains all basic network data structures.

use std::sync::atomic::AtomicU32;
use std::sync::Arc;
use std::collections::HashMap;
use std::sync::atomic::Ordering;

use bytemuck::Pod;
use bytemuck::Zeroable;
use glam::Vec2;
use tokio::sync::RwLock;
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

#[derive(Default)]
pub struct Network {
	pub allocation: NetworkAllocation,
	pub clip_count: AtomicU32,
	pub band_count: AtomicU32,
	pub lane_count: AtomicU32,
	pub vehicle_count: AtomicU32,
	pub vehicle_batch_counter: AtomicU32,
}

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

#[derive(Default, Clone)]
pub struct NetworkAllocation {
	pub clips: Arc<RwLock<HashMap<u32, Arc<RwLock<Clip>>>>>,
	pub bands: Arc<RwLock<HashMap<u32, Arc<RwLock<Band>>>>>,
	pub lanes: Arc<RwLock<HashMap<u32, Arc<RwLock<Lane>>>>>,
	pub vehicle_batches: Arc<RwLock<HashMap<u32, Arc<RwLock<VehicleBatch>>>>>,
	pub staged_vehicle_batch: Arc<RwLock<VehicleBatch>>,
	pub unused_vehicle_batchs: Arc<RwLock<Vec<Arc<RwLock<VehicleBatch>>>>>,
}

impl NetworkAllocation {
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