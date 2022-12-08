extern crate kiss3d;
use kiss3d::nalgebra as na;
use kiss3d::ncollide3d;

extern crate itertools;
extern crate itertools_num;

use std::rc::Rc;
use std::cell::RefCell;

use itertools_num::linspace;
use na::UnitQuaternion;
use na::Point3;

use kiss3d::resource::Mesh;
use kiss3d::light::Light;
use kiss3d::window::Window;

mod optics;
use optics::*;


fn draw_ray(ray: &Ray, window: &mut Window, length: F) {
    // Compute the end point of the ray
    let end_point = ray.origin + ray.direction.into_inner() * length;

    window.draw_line(
        &ray.origin.into(),
        &end_point.into(),
        &na::Point3::new(1.0, 0.0, 0.0),
    );
}

fn draw_trace(trace: &RayTrace, window: &mut Window, length: F) {
    if trace.points.len() > 0 {
        let mut last_point = trace.points[0];
        for point in trace.points.iter().skip(1) {
            window.draw_line(
                &last_point,
                &point,
                &na::Point3::new(1.0, 0.0, 0.0),
            );
            last_point = *point;
        }
        window.draw_line(
            &last_point,
            &(last_point + trace.direction.into_inner() * length),
            &na::Point3::new(1.0, 0.0, 0.0),
        );
    }
}

fn add_plane(plane: &Plane, size: F, color: &str, window: &mut Window) {
    // Compute the points at the corners of the lens
    let x = plane.normal.cross(&Vector::y()).normalize();
    let y = plane.normal.cross(&x).normalize();
    let top_left = plane.origin + (-0.5 * x + 0.5 * y) * size;
    let top_right = plane.origin + (0.5 * x + 0.5 * y) * size;
    let bottom_left = plane.origin + (-0.5 * x - 0.5 * y) * size;
    let bottom_right = plane.origin + (0.5 * x - 0.5 * y) * size;

    // Create a scene node to represent the lens
    let mut lens_node = window.add_quad_with_vertices(
        &[
            top_left.into(),
            top_right.into(),
            bottom_left.into(),
            bottom_right.into(),
        ],
        2, 2,
    );

    // Set the color of the lens
    lens_node.set_color(0.0, 1.0, 0.0);
}

fn add_surface(surface: &SphericalSurface, size: F, window: &mut Window) {
    let mut hemi : ncollide3d::procedural::TriMesh<f32> = ncollide3d::procedural::unit_hemisphere(16, 16);
    let pl = &surface.plane;
    let mut mat = Matrix3x3::from_columns(&[
        pl.tangent().into_inner(),
        pl.normal.into_inner(),
        pl.cotangent().into_inner(),
    ]);

    hemi.coords = hemi.coords.iter().map(|p| {
        mat * p * size + pl.origin.coords
    }).map(|p| {
        surface.tangent_plane(&p).origin
    }).collect();

    let mut mesh = Mesh::from_trimesh(hemi, false);
    let mut node = window.add_mesh(Rc::new(RefCell::new(mesh)), Vector::new(1.0, 1.0, 1.0));
    node.enable_backface_culling(false);
}

fn draw_axes(window: &mut Window) {
    window.draw_line(&Point3::origin(), &Point3::new(1.0, 0.0, 0.0), &Point3::new(1.0, 0.0, 0.0));
    window.draw_line(&Point3::origin(), &Point3::new(0.0, 1.0, 0.0), &Point3::new(0.0, 1.0, 0.0));
    window.draw_line(&Point3::origin(), &Point3::new(0.0, 0.0, 1.0), &Point3::new(0.0, 0.0, 1.0));
}

fn main() {
    let mut window = Window::new("Kiss3d: cube");

    window.set_light(Light::StickToCamera);

    let mut rays = Vec::new();

    for ang in linspace::<F>(0.0, 2.0*PI, 50) {
        rays.push(Ray::new(
            Point::new(0.0, ang.cos(), ang.sin()),
            Vector::new(1.0, 0.0, 0.0)
        ));
        rays.push(Ray::new(
            Point::new(0.0, 0.5*ang.cos(), 0.5*ang.sin()),
            Vector::new(1.0, 0.0, 0.0)
        ));
    }

    let lens = SphericalLens::new(
        Plane {
            origin: Point::new(1.0, 0.0, 0.0),
            normal: UnitVector::new_normalize(Vector::new(1.0, 0.5, 0.0)),
        },
        5.0,
        1.5,
        0.2,
    );
    add_surface(&lens.surface_in, 3.0, &mut window);
    add_surface(&lens.surface_out, 3.0, &mut window);

    let lens2 = SphericalLens::new(
        Plane {
            origin: Point::new(10.0, 0.0, 0.0),
            normal: UnitVector::new_normalize(Vector::new(1.0, 0.0, 0.0)),
        },
        5.0,
        1.5,
        0.2,
    );
    add_surface(&lens2.surface_in, 3.0, &mut window);
    add_surface(&lens2.surface_out, 3.0, &mut window);


    let mut optical_system = SequentialOpticalSystem::new();
    optical_system.surfaces.push(Box::new(lens.surface_in));
    optical_system.surfaces.push(Box::new(lens.surface_out));

    optical_system.surfaces.push(Box::new(lens2.surface_in));
    optical_system.surfaces.push(Box::new(lens2.surface_out));

    let traces: Vec<_> = rays.iter().map(|r| optical_system.trace_ray(&r)).collect();


    while window.render() {
        draw_axes(&mut window);
        traces.iter().for_each(|r| draw_trace(&r, &mut window, 1.0));
    }
}

