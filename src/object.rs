/*
use std::sync::{Arc, atomic::{AtomicUsize, Ordering}, RwLock};

use vulkano::{buffer::{CpuAccessibleBuffer, BufferUsage}, device::Device, command_buffer::{AutoCommandBufferBuilder, PrimaryAutoCommandBuffer}};

use crate::network::NetworkVertex;

// Use Arc::get_mut_unchecked to access your Arc<NetworkObject> then
// write to cpu data. If the cpu data is written on multiple threads,
// then use an RwLock on the cpu data, but never the network object.
//
// Only call sub functions (to write buffers with cpu data) on the same
// thread that you are writing the cpu data on. If this is a different
// thread, then the cpu data must be wrapped in an RwLock.
#[derive(Default)]
pub struct NetworkObjectCpuData {
	pub cpu_vertices: Vec<NetworkVertex>,
	pub cpu_indices: Vec<u32>,
}

pub struct NetworkObject {
	pub cpu_data: Arc<RwLock<NetworkObjectCpuData>>,
	pub vertex_buffer: Arc<CpuAccessibleBuffer<[NetworkVertex]>>,
	pub index_buffer: Arc<CpuAccessibleBuffer<[u32]>>,
	pub vertex_capacity: AtomicUsize,
	pub index_capacity: AtomicUsize,
	pub index_buffer_len: AtomicUsize,
}

impl NetworkObject {
	pub fn new(device: &Arc<Device>, vertex_capacity: usize, index_capacity: usize) -> Self {
		NetworkObject {
			cpu_data: Arc::new(RwLock::new(NetworkObjectCpuData {
				cpu_vertices: Vec::with_capacity(vertex_capacity),
				cpu_indices: Vec::with_capacity(index_capacity)
			})),
			vertex_buffer: CpuAccessibleBuffer::from_iter(
				device.clone(),
				BufferUsage {
					vertex_buffer: true,
					..BufferUsage::empty()
				},
				false,
				vec![NetworkVertex::default(); vertex_capacity]
			).unwrap(),
			index_buffer: CpuAccessibleBuffer::from_iter(
				device.clone(),
				BufferUsage {
					index_buffer: true,
					..BufferUsage::empty()
				},
				false,
				vec![0u32; index_capacity]
			).unwrap(),
			vertex_capacity: AtomicUsize::new(vertex_capacity),
			index_capacity: AtomicUsize::new(index_capacity),
			index_buffer_len: AtomicUsize::new(0),
		}
	}

	pub fn fill_overwrite(&mut self) {
		let ra_cpu_data = self.cpu_data.read().unwrap();
		if ra_cpu_data.cpu_vertices.len() > self.vertex_capacity.load(Ordering::SeqCst) ||
			ra_cpu_data.cpu_indices.len() > self.index_capacity.load(Ordering::SeqCst) {
			panic!("cpu data excedes declared capacity; failed to overwrite buffer");
		}
		let mut wa_vb = self.vertex_buffer.write().expect("failed to lock vehicle_vertex_buffer");
		let mut wa_ib = self.index_buffer.write().expect("failed to lock vehicle_index_buffer");
		wa_vb.fill(NetworkVertex::default());
		wa_ib.fill(0);
		for vertex in ra_cpu_data.cpu_vertices.iter().enumerate() {
			wa_vb[vertex.0] = vertex.1.clone();
		}
		for index in ra_cpu_data.cpu_indices.iter().enumerate() {
			wa_ib[index.0] = index.1.clone();
		}
		self.index_buffer_len.store(ra_cpu_data.cpu_indices.len(), Ordering::SeqCst);
	}

	pub fn render(
		&self,
		builder: &mut AutoCommandBufferBuilder<PrimaryAutoCommandBuffer>
	) {
		builder.bind_vertex_buffers(
			0,
			self.vertex_buffer.clone()
		).bind_index_buffer(
			self.index_buffer.clone()
		).draw_indexed(
			self.index_buffer_len.load(Ordering::SeqCst) as u32,
			1, 0, 0, 0
		).unwrap();
	}
}
*/