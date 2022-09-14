use std::collections::VecDeque;
use std::ops::Add;
use std::sync::RwLockWriteGuard;

use bevy::input::mouse::{MouseMotion, MouseWheel};
use bevy::render::camera::Projection;
use bevy::transform;
use bevy::{prelude::*, sprite::MaterialMesh2dBundle};

pub mod components;
use components::*;

pub mod resources;
use resources::*;
use resources::network::*;
use smooth_bevy_cameras::controllers::fps::{FpsCameraPlugin, FpsCameraBundle, FpsCameraController};
use smooth_bevy_cameras::controllers::orbit::{OrbitCameraPlugin, OrbitCameraBundle, OrbitCameraController};
use smooth_bevy_cameras::{LookTransformBundle, LookTransform, Smoother, LookTransformPlugin};

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
	
	let band_a = spawn_band(&mut w_network, clip_a).unwrap();
	let band_b = spawn_band(&mut w_network, clip_a).unwrap();
	let band_c = spawn_band(&mut w_network, clip_a).unwrap();
	let band_d = spawn_band(&mut w_network, clip_b).unwrap();
	let band_e = spawn_band(&mut w_network, clip_d).unwrap();
	let band_f = spawn_band(&mut w_network, clip_e).unwrap();
	let band_g = spawn_band(&mut w_network, clip_d).unwrap();
	let band_h = spawn_band(&mut w_network, clip_g).unwrap();
	let band_i = spawn_band(&mut w_network, clip_f).unwrap();
	let band_j = spawn_band(&mut w_network, clip_i).unwrap();
	
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
		clip_b, clip_c, 0, 0, band_c
	).unwrap();

	// SQUIGLE

	let lane_dg = spawn_lane(&mut w_network, &mut commands, &mut meshes, &mut materials,
		Vec2::new(3.0, spread * 1.0),
		Vec2::new(3.0, spread * 2.2),
		Vec2::new(6.0, spread * 1.8),
		Vec2::new(6.0, spread * 3.0),
		clip_b, clip_d, 1, 2, band_a
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
		clip_e, clip_f, 0, 1, band_f
	).unwrap();
	let lane_n = spawn_lane_streight(&mut w_network, &mut commands, &mut meshes, &mut materials,
		Vec2::new(3.0, spread * 4.0),
		Vec2::new(3.0, spread * 5.0),
		clip_e, clip_f, 1, 1, band_f
	).unwrap();
	let lane_o = spawn_lane_streight(&mut w_network, &mut commands, &mut meshes, &mut materials,
		Vec2::new(6.0, spread * 4.0),
		Vec2::new(6.0, spread * 5.0),
		clip_e, clip_f, 2, 2, band_f
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

	spawn_vehicle(&mut w_network, &mut commands, &mut meshes, &mut materials, clip_a, band_a, lane_a);

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
	let network = network.network.read().unwrap();
	for (mut vehicle, mut transform) in &mut query {
		vehicle.distance += vehicle.speed * time.delta_seconds();
		vehicle.speed += vehicle.acceleration * time.delta_seconds();
		let mut lane = match network.alloc_lanes.get(&vehicle.active_lane) {
			Some(x) => x,
			None => {
				commands.entity(vehicle.entity).despawn();
				println!("despawning: active lane is invalid");
				continue;
			},
		};
		
		while vehicle.distance >= lane.length {
			vehicle.distance = vehicle.distance - lane.length;
			if let Some(x) = vehicle.forward_lanes.pop_front() {
				vehicle.forward_length = vehicle.forward_length - x.1;
			}

			if vehicle.navigation.nav.is_empty() {
				if !vehicle.navigation.renavigate() {
					commands.entity(vehicle.entity).despawn();
					println!("despawning: navigation empty");
					break;
				}
			}
			
			let new_forward = vehicle.navigation.get_forward_lanes(
				&network,
				500.0 - vehicle.forward_length,
				match vehicle.forward_lanes.back() {
					Some(x) => x.0,
					None => vehicle.active_lane,
				},
				vehicle.navigation.recent_nav + (vehicle.forward_lanes.len() as u16)
			);
			
			for fl in new_forward.iter() {
				vehicle.forward_length = vehicle.forward_length + fl.1;
				vehicle.forward_lanes.push_back(*fl);
			}

			if vehicle.forward_lanes.is_empty() {
				commands.entity(vehicle.entity).despawn();
				println!("despawning: no forward lanes");
				break;
			}

			if vehicle.navigation.nav[(vehicle.navigation.recent_nav) as usize].clip != vehicle.active_clip ||
				vehicle.navigation.nav[(vehicle.navigation.recent_nav) as usize].band != vehicle.active_band ||
				vehicle.navigation.nav_valid_band_lanes[(vehicle.navigation.recent_nav) as usize].iter().find(|&&x| x == vehicle.active_lane) == None {
				
				if !vehicle.navigation.renavigate() {
					commands.entity(vehicle.entity).despawn();
					println!("despawning: invalid navigation");
					break;
				}
			}

			vehicle.navigation.recent_nav = vehicle.navigation.recent_nav + 1;
			vehicle.active_lane = vehicle.forward_lanes.pop_front().unwrap().0;
			let vehicle_lane = match network.alloc_lanes.get(&vehicle.active_lane) {
				Some(x) => x,
				None => { panic!("um"); },
			};
			vehicle.active_band = vehicle_lane.identity.band;
			vehicle.active_clip = vehicle_lane.identity.clip;

			lane = match network.alloc_lanes.get(&vehicle.active_lane) {
				Some(x) => x,
				None => {
					commands.entity(vehicle.entity).despawn();
					println!("despawning: active lane invalid 2");
					break;
				},
			};
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
	network.alloc_clips.insert(clip_count, Clip{ lanes_fixed: Vec::new() });
	Some(clip_count)
}

fn spawn_band(
	network: &mut RwLockWriteGuard<Network>,
	clip_id: u32
) -> Option<u32> {
	let band_count = network.band_count + 1;
	network.lane_count = band_count;
	network.alloc_bands.insert(band_count, Band{ lanes: Vec::new() });
	Some(band_count)
}

fn spawn_lane_streight(
	network: &mut RwLockWriteGuard<Network>,
	commands: &mut Commands,
	meshes: &mut ResMut<Assets<Mesh>>,
	materials: &mut ResMut<Assets<StandardMaterial>>,
	p1: Vec2, p2: Vec2,
	clip_bw: u32, clip_fw: u32, lnum_bw: u32, lnum_fw: u32, band: u32
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
	clip_bw: u32, clip_fw: u32, lnum_bw: u32, lnum_fw: u32, band: u32
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
	});

	{
		let bw_lanes = &mut network.alloc_clips.get_mut(&clip_bw).expect("invalid clip").lanes_fixed;
		bw_lanes.resize((lnum_bw + 1) as usize, (0, 0));
		bw_lanes[lnum_bw as usize].0 = lane_count;
	} {
		let fw_lanes = &mut network.alloc_clips.get_mut(&clip_fw).expect("invalid clip").lanes_fixed;
		fw_lanes.resize((lnum_fw + 1) as usize, (0, 0));
		fw_lanes[lnum_fw as usize].1 = lane_count;
	}

	let bw_lane = network.alloc_clips.get(&clip_bw).expect("invalid clip").lanes_fixed[lnum_bw as usize].1;
	match network.alloc_lanes.get_mut(&bw_lane) {
		Some(x) => {
			x.fw.push(NetworkIdentifier { lane: lane_count, band, clip: clip_bw });
		},
		None => {},
	};

	let fw_lane = network.alloc_clips.get(&clip_fw).expect("invalid clip").lanes_fixed[lnum_fw as usize].0;
	match network.alloc_lanes.get_mut(&fw_lane) {
		Some(x) => {
			x.bw.push(NetworkIdentifier { lane: lane_count, band, clip: clip_bw /* this lane's clip; not the forward lane's clip */ });
		},
		None => {},
	};

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
	network: &mut RwLockWriteGuard<Network>,
	commands: &mut Commands,
	meshes: &mut ResMut<Assets<Mesh>>,
	materials: &mut ResMut<Assets<StandardMaterial>>,
	clip_src: u32, band_src: u32, lane_src: u32,
	clip_dst: u32, band_dst: u32, lane_dst: u32
) {
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
		navigation: Navigation { recent_nav: 0, nav: Vec::new(), nav_valid_band_lanes: Vec::new() },
		forward_lanes: fw_lanes,
		forward_length: lane.length,
	};

	let new_forward = vehicle_comp.navigation.get_forward_lanes_mut(
		network,
		500.0 - vehicle_comp.forward_length,
		match vehicle_comp.forward_lanes.back() {
			Some(x) => x.0,
			None => vehicle_comp.active_lane,
		},
		vehicle_comp.navigation.recent_nav + (vehicle_comp.forward_lanes.len() as u16)
	);
	
	for fl in new_forward.iter() {
		vehicle_comp.forward_length = vehicle_comp.forward_length + fl.1;
		vehicle_comp.forward_lanes.push_back(*fl);
	}

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