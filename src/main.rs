use std::collections::VecDeque;
use std::ops::Add;
use std::sync::{RwLockWriteGuard, RwLock};

use bevy::input::mouse::{MouseMotion, MouseWheel};
use bevy::render::camera::Projection;
use bevy::transform;
use bevy::{prelude::*, sprite::MaterialMesh2dBundle};

pub mod components;
use components::*;

pub mod resources;
use kernel::Kernel;
use resources::*;
use resources::network::*;
use smooth_bevy_cameras::controllers::fps::{FpsCameraPlugin, FpsCameraBundle, FpsCameraController};
use smooth_bevy_cameras::controllers::orbit::{OrbitCameraPlugin, OrbitCameraBundle, OrbitCameraController};
use smooth_bevy_cameras::{LookTransformBundle, LookTransform, Smoother, LookTransformPlugin};

pub mod kernel;

fn main() {
	App::new()
		.add_plugins(DefaultPlugins)
		.add_plugin(LookTransformPlugin)
		.add_plugin(OrbitCameraPlugin::default())
		.add_plugin(TEPlugin)
    	.run();
}

pub struct TEPlugin;

impl Plugin for TEPlugin {
    fn build(&self, app: &mut App) {
        // app.insert_resource(GreetTimer(Timer::from_seconds(2.0, true)))
		app.init_resource::<NetworkResource>()
			.add_startup_system(setup)
        	.add_system(vehicle_system);
    }
}

fn setup(
	mut commands: Commands,
	mut meshes: ResMut<Assets<Mesh>>,
	mut materials: ResMut<Assets<StandardMaterial>>,
	mut network: ResMut<NetworkResource>
) {
	let mut cam_controller = OrbitCameraController::default();
	cam_controller.mouse_translate_sensitivity *= 10.0;

	let cam_bundle = OrbitCameraBundle::new(
		cam_controller,
		Vec3::new(0.0, 0.0, 30.0),
		Vec3::new(0.0, 0.0, 0.0),
	);
	commands.spawn_bundle(Camera3dBundle::default()).insert_bundle(cam_bundle);

	let mut directional_light = DirectionalLight::default();
	directional_light.color = Color::rgb(1.0, 0.9, 0.8);
	directional_light.illuminance *= 0.8;	
	commands.spawn_bundle(DirectionalLightBundle{
		directional_light,
		transform: Transform::from_rotation(Quat::from_euler(EulerRot::XYZ, 20.0, 70.0, 0.0)),
		..default()
	});

	let spread: f32 = 5.0;

	let mut w_network = network.network.write().unwrap();
	
	let clip_a = spawn_clip(&mut w_network).unwrap();
	let clip_b = spawn_clip(&mut w_network).unwrap();
	let clip_c = spawn_clip(&mut w_network).unwrap();
	let clip_d = spawn_clip(&mut w_network).unwrap();
	let clip_e = spawn_clip(&mut w_network).unwrap();
	let clip_f = spawn_clip(&mut w_network).unwrap();
	let clip_g = spawn_clip(&mut w_network).unwrap();
	let clip_h = spawn_clip(&mut w_network).unwrap();
	let clip_i = spawn_clip(&mut w_network).unwrap();
	let clip_j = spawn_clip(&mut w_network).unwrap();
	
	let band_a = spawn_band(&mut w_network, clip_a, clip_b).unwrap();
	let band_b = spawn_band(&mut w_network, clip_b, clip_c).unwrap();
	let band_c = spawn_band(&mut w_network, clip_c, clip_d).unwrap();
	let band_d = spawn_band(&mut w_network, clip_b, clip_d).unwrap();
	let band_e = spawn_band(&mut w_network, clip_d, clip_e).unwrap();
	let band_f = spawn_band(&mut w_network, clip_e, clip_f).unwrap();
	let band_g = spawn_band(&mut w_network, clip_d, clip_g).unwrap();
	let band_h = spawn_band(&mut w_network, clip_g, clip_h).unwrap();
	let band_i = spawn_band(&mut w_network, clip_f, clip_i).unwrap();
	let band_j = spawn_band(&mut w_network, clip_i, clip_j).unwrap();
	
	// FIRST STREIGHT

	let lane_a = spawn_lane_streight(&mut w_network, &mut commands, &mut meshes, &mut materials,
		Vec2::new(0.0, spread * 0.0),
		Vec2::new(0.0, spread * 1.0),
		clip_a, clip_b, 0, 0, band_a
	).unwrap();
	let lane_b = spawn_lane_streight(&mut w_network, &mut commands, &mut meshes, &mut materials,
		Vec2::new(3.0, spread * 0.0),
		Vec2::new(3.0, spread * 1.0),
		clip_a, clip_b, 1, 1, band_a
	).unwrap();
	let lane_c = spawn_lane_streight(&mut w_network, &mut commands, &mut meshes, &mut materials,
		Vec2::new(0.0, spread * 1.0),
		Vec2::new(0.0, spread * 2.0),
		clip_b, clip_c, 0, 0, band_b
	).unwrap();

	// SQUIGLE

	let lane_dg = spawn_lane(&mut w_network, &mut commands, &mut meshes, &mut materials,
		Vec2::new(3.0, spread * 1.0),
		Vec2::new(3.0, spread * 2.2),
		Vec2::new(6.0, spread * 1.8),
		Vec2::new(6.0, spread * 3.0),
		clip_b, clip_d, 1, 2, band_d
	).unwrap();

	// EXPAND

	let lane_e = spawn_lane_streight(&mut w_network, &mut commands, &mut meshes, &mut materials,
		Vec2::new(0.0, spread * 2.0),
		Vec2::new(0.0, spread * 3.0),
		clip_c, clip_d, 0, 0, band_c
	).unwrap();
	let lane_f = spawn_lane_streight(&mut w_network, &mut commands, &mut meshes, &mut materials,
		Vec2::new(0.0, spread * 2.0),
		Vec2::new(3.0, spread * 3.0),
		clip_c, clip_d, 0, 1, band_c
	).unwrap();

	// STREIGHT AWAY

	let lane_i = spawn_lane_streight(&mut w_network, &mut commands, &mut meshes, &mut materials,
		Vec2::new(0.0, spread * 3.0),
		Vec2::new(0.0, spread * 4.0),
		clip_d, clip_e, 0, 0, band_e
	).unwrap();
	let lane_j = spawn_lane_streight(&mut w_network, &mut commands, &mut meshes, &mut materials,
		Vec2::new(3.0, spread * 3.0),
		Vec2::new(3.0, spread * 4.0),
		clip_d, clip_e, 1, 1, band_e
	).unwrap();
	let lane_k = spawn_lane_streight(&mut w_network, &mut commands, &mut meshes, &mut materials,
		Vec2::new(6.0, spread * 3.0),
		Vec2::new(6.0, spread * 4.0),
		clip_d, clip_e, 2, 2, band_e
	).unwrap();

	// MERGE A

	let lane_m = spawn_lane_streight(&mut w_network, &mut commands, &mut meshes, &mut materials,
		Vec2::new(0.0, spread * 4.0),
		Vec2::new(3.0, spread * 5.0),
		clip_e, clip_f, 0, 0, band_f
	).unwrap();
	let lane_n = spawn_lane_streight(&mut w_network, &mut commands, &mut meshes, &mut materials,
		Vec2::new(3.0, spread * 4.0),
		Vec2::new(3.0, spread * 5.0),
		clip_e, clip_f, 1, 0, band_f
	).unwrap();
	let lane_o = spawn_lane_streight(&mut w_network, &mut commands, &mut meshes, &mut materials,
		Vec2::new(6.0, spread * 4.0),
		Vec2::new(6.0, spread * 5.0),
		clip_e, clip_f, 2, 1, band_f
	).unwrap();

	// MERGE INT

	let lane_p = spawn_lane_streight(&mut w_network, &mut commands, &mut meshes, &mut materials,
		Vec2::new(3.0, spread * 5.0),
		Vec2::new(3.0, spread * 6.0),
		clip_f, clip_i, 0, 0, band_i
	).unwrap();
	let lane_q = spawn_lane_streight(&mut w_network, &mut commands, &mut meshes, &mut materials,
		Vec2::new(6.0, spread * 5.0),
		Vec2::new(6.0, spread * 6.0),
		clip_f, clip_i, 1, 1, band_i
	).unwrap();

	// MERGE B

	let lane_r = spawn_lane_streight(&mut w_network, &mut commands, &mut meshes, &mut materials,
		Vec2::new(3.0, spread * 6.0),
		Vec2::new(3.0, spread * 7.0),
		clip_i, clip_j, 0, 0, band_j
	).unwrap();
	let lane_s = spawn_lane_streight(&mut w_network, &mut commands, &mut meshes, &mut materials,
		Vec2::new(6.0, spread * 6.0),
		Vec2::new(3.0, spread * 7.0),
		clip_i, clip_j, 1, 0, band_j
	).unwrap();

	// EXIT

	let lane_h = spawn_lane(&mut w_network, &mut commands, &mut meshes, &mut materials,
		Vec2::new(0.0, spread * 3.0),
		Vec2::new(0.0, spread * 3.2),
		Vec2::new(0.0, spread * 3.7),
		Vec2::new(-4.0, spread * 4.0),
		clip_d, clip_g, 0, 0, band_g
	).unwrap();

	// VEHICLES

	drop(w_network);

	println!("spawning veh {} {} {}", clip_a, band_a, lane_a);

	spawn_vehicle(&mut network.network, &mut commands, &mut meshes, &mut materials,
		clip_a, band_a, lane_a,
		clip_i, band_j, lane_r
	);

}

fn get_primary_window_size(windows: &Res<Windows>) -> Vec2 {
	let window = windows.get_primary().unwrap();
	let window = Vec2::new(window.width() as f32, window.height() as f32);
	window
}

fn vehicle_system(
	time: Res<Time>,
	mut network: ResMut<NetworkResource>,
	mut query: Query<(&mut VehicleComponent, &mut Transform)>,
	mut commands: Commands
) {
	let mut network = network.network.write().unwrap();
	for (mut vehicle, mut transform) in &mut query {
		vehicle.distance += vehicle.speed * time.delta_seconds();
		vehicle.speed += vehicle.acceleration * time.delta_seconds();

		//

		if vehicle.forward_lanes.len() <= 1 {
			let clip = network.alloc_clips.get(&vehicle.active_clip).expect("invalid clip");
			let mut best_id: u32 = 0;
			let mut best_diff: i8 = i8::MAX;
			let fixed_current = clip.lanes_fixed.iter().enumerate().find(
				|x|
				x.1.1 == vehicle.active_lane
			).expect("invalid lane id");

			for lane_id in vehicle.navigation.nav_valid_band_lanes[vehicle.navigation.recent_nav as usize].iter() {
				if let Some(fixed_valid) = clip.lanes_fixed.iter().enumerate().find(
					|x|
					x.1.1 == *lane_id
				) {
					let diff_abs: i8 = ((fixed_valid.0 as i8) - (fixed_current.0 as i8)).abs();
					if diff_abs < best_diff as i8 {
						best_diff = diff_abs;
						best_id = *lane_id;
					}
				}
			}

			if best_id > 0 && best_id != vehicle.active_lane {
				vehicle.active_lane = best_id;
				println!("moving to best lane");
			}
		}

		let mut lane = match network.alloc_lanes.get_mut(&vehicle.active_lane) {
			Some(x) => x,
			None => {
				commands.entity(vehicle.entity).despawn();
				println!("despawning: active lane is invalid");
				continue;
			},
		};
		
		while vehicle.distance >= lane.length {
			if let Some(idx) = lane.vehicles.iter().position(|x| x.0 == vehicle.id) {
				lane.vehicles.swap_remove(idx);
			} else {
				println!("failed to remove vehicle from lane; id does not exist in specified lane");
			}

			vehicle.distance -= lane.length;

			if vehicle.forward_lanes.is_empty() {
				// let a = &mut *network;
				vehicle.pull_forward_lanes(&*network);
				println!("{:?}", vehicle.forward_lanes);
			}

			let fw_lane = match vehicle.forward_lanes.pop_front() {
				Some(x) => x,
				None => {
					commands.entity(vehicle.entity).despawn();
					println!("despawning: no forward lanes ");
					break;
				}
			};

			vehicle.active_lane = fw_lane.0;
			lane = network.alloc_lanes.get_mut(&vehicle.active_lane).expect("invalid forward lane");
			lane.vehicles.push((vehicle.id, vehicle.distance));
			vehicle.active_band = lane.identity.band;
			vehicle.active_clip = lane.identity.clip;
			vehicle.forward_length -= fw_lane.1;
			vehicle.navigation.recent_nav += 1;
			println!("inc recent nav to {}", vehicle.navigation.recent_nav);

			let v_clip = vehicle.active_clip;
			let v_band = vehicle.active_band;
			let v_lane = vehicle.active_lane;

			// vehicle.pull_forward_lanes(&network);

			if vehicle.navigation.nav[(vehicle.navigation.recent_nav) as usize].clip != v_clip ||
				vehicle.navigation.nav[(vehicle.navigation.recent_nav) as usize].band != v_band ||
				vehicle.navigation.nav_valid_band_lanes[(vehicle.navigation.recent_nav) as usize].iter().find(|&&x| x == v_lane) == None {
				
				// if !vehicle.navigation.renavigate(&network, v_clip, v_band, v_lane) {
				// 	commands.entity(vehicle.entity).despawn();
				// 	println!("despawning: invalid navigation");
				// 	break;
				// }
			}

			// match vehicle.
		}
		
		let v_pos = lane.get_interp_position(vehicle.distance);

		// transform.rotation = Quat::from_rotation_z(0.05);
		transform.translation = Vec3::new(v_pos.x, v_pos.y, 1.0);
	}
}

fn spawn_clip(
	network: &mut RwLockWriteGuard<Network>,
) -> Option<u32> {
	let clip_count = network.clip_count + 1;
	network.clip_count = clip_count;
	network.alloc_clips.insert(clip_count, Clip { lanes_fixed: Vec::new(), fw_bands: Vec::new() });
	Some(clip_count)
}

fn spawn_band(
	network: &mut RwLockWriteGuard<Network>,
	src_clip: u32, dst_clip: u32
) -> Option<u32> {
	let band_count = network.band_count + 1;
	network.band_count = band_count;
	network.alloc_bands.insert(band_count, Band { src_clip, src_min: u8::MAX, src_max: u8::MAX, dst_clip, dst_min: u8::MAX, dst_max: u8::MAX, empty: true });
	Some(band_count)
}

fn spawn_lane_streight(
	network: &mut RwLockWriteGuard<Network>,
	commands: &mut Commands,
	meshes: &mut ResMut<Assets<Mesh>>,
	materials: &mut ResMut<Assets<StandardMaterial>>,
	p1: Vec2, p2: Vec2,
	clip_bw: u32, clip_fw: u32, lnum_bw: u8, lnum_fw: u8, band: u32
) -> Option<u32> {
	let control = (p2 - p1) * 0.1;
	spawn_lane(
		network,
		commands,
		meshes,
		materials,
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

fn spawn_lane(
	network: &mut RwLockWriteGuard<Network>,
	commands: &mut Commands,
	meshes: &mut ResMut<Assets<Mesh>>,
	materials: &mut ResMut<Assets<StandardMaterial>>,
	p1: Vec2, p2: Vec2, p3: Vec2, p4: Vec2,
	clip_bw: u32, clip_fw: u32, lnum_bw: u8, lnum_fw: u8, band: u32
) -> Option<u32> {
	let lane_count = network.lane_count + 1;
	network.lane_count = lane_count;

	let mut point_entities: Vec<Entity> = Vec::new();
	let mut points: Vec<(Vec2, f32)> = Vec::new();

	let mut last_point = p1;
	let mut total_distance: f32 = 0.0;

	for i in 0..10 {
		let t: f32 = (i as f32) / 9.0;
		let omt: f32 = 1.0 - t;

		let tm1: Vec2 = p1 * omt.powf(3.0);
		let tm2: Vec2 = p2 * omt.powf(2.0) * t * 3.0;
		let tm3: Vec2 = p3 * omt * t.powf(2.0) * 3.0;
		let tm4: Vec2 = p4 * t.powf(3.0);

		let p: Vec2 = tm1 + tm2 + tm3 + tm4;
		let mut dis: f32 = 0.0;

		if i > 0 {
			dis = last_point.distance(p);
		}

		last_point = p;
		total_distance = total_distance + dis;
		points.push((Vec2::new(p.x, p.y), total_distance));

		point_entities.push(commands.spawn_bundle(PbrBundle {
			mesh: meshes.add(Mesh::from(shape::Cube { size: 0.5 })),
			material: materials.add(Color::rgb(0.5, 0.5, 0.5).into()),
			transform: Transform::from_xyz(p.x, p.y, 0.0),
			..default()
		}).id());
	}

	let entity = commands.spawn().insert(LaneComponent{}).id();

	network.alloc_lanes.insert(lane_count, Lane {
		p1,
		p2,
		p3,
		p4,
		points,
		identity: LaneIdentifier { band, clip: clip_bw },
		fw: Vec::new(),
		bw: Vec::new(),
		length: total_distance,
		vehicles: Vec::new(),
	});

	{
		// println!("cbw {}", clip_bw);
		let bw_lanes = &mut network.alloc_clips.get_mut(&clip_bw).expect("invalid clip").lanes_fixed;
		// println!("bw lanes {:?}", bw_lanes);
		if bw_lanes.len() < (lnum_bw + 1) as usize {
			bw_lanes.resize((lnum_bw + 1) as usize, (0, 0));
		}
		bw_lanes[lnum_bw as usize].1 = lane_count;
		// println!("bw lanes {:?}", bw_lanes);
	} {
		// println!("cfw {}", clip_fw);
		let fw_lanes = &mut network.alloc_clips.get_mut(&clip_fw).expect("invalid clip").lanes_fixed;
		// println!("fw lanes {:?}", fw_lanes);
		if fw_lanes.len() < (lnum_fw + 1) as usize {
			fw_lanes.resize((lnum_fw + 1) as usize, (0, 0));
		}
		fw_lanes[lnum_fw as usize].0 = lane_count;
		// println!("fw lanes {:?}", fw_lanes);
	}

	let bw_lane = network.alloc_clips.get(&clip_bw).expect("invalid clip").lanes_fixed[lnum_bw as usize].0;
	match network.alloc_lanes.get_mut(&bw_lane) {
		Some(x) => {
			x.fw.push(NetworkIdentifier { lane: lane_count, band, clip: clip_bw });
		},
		None => {},
	};

	let fw_lane = network.alloc_clips.get(&clip_fw).expect("invalid clip").lanes_fixed[lnum_fw as usize].1;
	match network.alloc_lanes.get_mut(&fw_lane) {
		Some(x) => {
			x.bw.push(NetworkIdentifier { lane: lane_count, band, clip: clip_bw /* this lane's clip; not the forward lane's clip */ });
		},
		None => {},
	};

	let mut lband = network.alloc_bands.get_mut(&band).expect("invalid band");
	if lband.empty {
		lband.empty = false;
		lband.src_min = lnum_bw;
		lband.src_max = lnum_bw;
		lband.dst_min = lnum_fw;
		lband.dst_max = lnum_fw;
	} else {
		lband.src_min = lband.src_min.min(lnum_bw);
		lband.src_max = lband.src_max.max(lnum_bw);
		lband.dst_min = lband.src_min.min(lnum_fw);
		lband.dst_max = lband.src_max.max(lnum_fw);
	}

	println!("c {} given fw band {}", clip_bw, band);

	let fw_bands = &mut network.alloc_clips.get_mut(&clip_bw).expect("invalid clip").fw_bands;
	if !fw_bands.contains(&band) {
		fw_bands.push(band);
	}


	// .push((
	// 	BandNode{ min: , max: todo!()  }
	// ))

	// for lane_idx in fw_fixed {
	// 	// TODO: bands
	// 	match network.alloc_lanes.get(lane_idx) {
	// 		Some(x) => x,
	// 		None => { continue; },
	// 	}
	// }

	Some(lane_count)
}

fn spawn_vehicle(
	network_l: &mut RwLock<Network>,
	commands: &mut Commands,
	meshes: &mut ResMut<Assets<Mesh>>,
	materials: &mut ResMut<Assets<StandardMaterial>>,
	clip_src: u32, band_src: u32, lane_src: u32,
	clip_dst: u32, band_dst: u32, lane_dst: u32
) {
	let mut network = network_l.write().unwrap();
	let vehicle_count = network.vehicle_count + 1;
	network.vehicle_count = vehicle_count;

	let lane = network.alloc_lanes.get(&lane_src).expect("invalid lane");
	let mut spawn = commands.spawn();

	let vehicle = spawn.insert_bundle(PbrBundle {
		mesh: meshes.add(Mesh::from(shape::Cube { size: 2.5 })),
		material: materials.add(Color::rgb(0.2, 0.0, 0.9).into()),
		transform: Transform::from_xyz(lane.p1.x, lane.p1.y, 0.0),
		..default()
	});

	let mut fw_lanes = VecDeque::new();
	fw_lanes.push_back((lane_src, lane.length));

	let mut vehicle_comp = VehicleComponent{
		id: vehicle_count,
		speed: 5.0,
		acceleration: 0.0,
		target: VTarget::AccFStop,
		stage: VStage::Wait,
		active_clip: clip_src,
		active_band: band_src,
		active_lane: lane_src,
   	distance: 0.0,
		entity: vehicle.id(),
		navigation: Navigation {
			recent_nav: 0,
			nav: Vec::new(),
			nav_valid_band_lanes: Vec::new(),
			target_clip: clip_dst,
			target_band: band_dst,
			target_lane: lane_dst
		},
		forward_lanes: VecDeque::new(),//fw_lanes,
		forward_length: 0.0,//lane.length,
		kernel: Kernel {
			wait_exp_0: 1.0,
			wait_exp_1: 3.0,
			wait_releases: 0x1 | 0x2 | 0x4 | 0x8 | 0x10 | 0x20,
},
	};

	drop(network);
	let network = network_l.read().unwrap();

	vehicle_comp.navigation.renavigate(&network, clip_src, band_src, lane_src);

	vehicle.insert(vehicle_comp);
}

// struct GreetTimer(Timer);

// fn greet_people(time: Res<Time>, mut timer: ResMut<GreetTimer>, query: Query<&Name, With<Person>>) {
// 	if timer.0.tick(time.delta()).just_finished() {
// 		for name in query.iter() {
// 			println!("hello {}!", name.0);
// 		}
// 	}
// }