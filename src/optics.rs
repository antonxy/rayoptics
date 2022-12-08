use kiss3d::nalgebra as na;
use itertools_num::linspace;
use na::UnitQuaternion;
use na::Point3;

pub type F = f32;
pub use std::f32::consts::PI;

pub type Vector = na::Vector3<F>;
pub type UnitVector = na::UnitVector3<F>;
pub type Point = na::Point3<F>;
pub type Matrix3x3 = na::SMatrix<F, 3, 3>;

pub trait OpticalSurface {
    fn ray_interaction(&self, ray: &Ray) -> Option<Ray>;
}

#[derive(Clone, Debug)]
pub struct Plane {
    pub origin: Point,
    pub normal: UnitVector,
}

impl Plane {
    pub fn new(origin: Point, normal: Vector) -> Self {
        Self {
            origin: origin,
            normal: UnitVector::new_normalize(normal),
        }
    }

    pub fn offset(&self, offset: F) -> Self {
        Self {
            origin: self.origin + self.normal.into_inner() * offset,
            normal: self.normal
        }
    }

    pub fn flip(&self) -> Self {
        Self {
            origin: self.origin,
            normal: -self.normal
        }
    }

    pub fn refract_vector(&self, direction: &UnitVector, relative_refractive_index: F) -> Option<UnitVector> {
        let eta = 1.0/relative_refractive_index;
        let n = &-self.normal.into_inner();
        let i = &direction.into_inner();

        let k = 1.0 - eta * eta * (1.0 - n.dot(i) * n.dot(i));
        if k < 0.0 {
            None // Total reflection
        } else {
            Some(UnitVector::new_normalize(eta * i - (eta * n.dot(i) + k.sqrt()) * n))
        }
    }

    pub fn tangent(&self) -> UnitVector {
        if self.normal.dot(&Vector::x()).abs() > 0.9 {
            UnitVector::new_normalize(Vector::y().cross(&self.normal))
        } else {
            UnitVector::new_normalize(Vector::x().cross(&self.normal))
        }
    }

    pub fn cotangent(&self) -> UnitVector {
        UnitVector::new_normalize(self.normal.cross(&self.tangent()))
    }
}

#[derive(Clone, Debug)]
pub struct Ray {
    pub origin: Point,
    pub direction: UnitVector,
}

impl Ray {
    pub fn new(origin: Point, direction: Vector) -> Self {
        Self {
            origin: origin,
            direction: UnitVector::new_normalize(direction),
        }
    }

    pub fn intersect(&self, plane: &Plane) -> Option<Point> {
        let d = self.direction.dot(&plane.normal);
        if d == 0.0 {
            return None;
        }

        let t = (plane.origin - self.origin).dot(&plane.normal) / d;

        // We only want intersections in the positive direction of the ray
        if t < 0.0 {
            return None;
        }

        let intersection = self.origin + t * self.direction.into_inner();

        return Some(intersection);
    }
}

// Only works paraxial I think. Better implement a spherical lens or sth.
pub struct IdealLens {
    pub plane: Plane,
    pub focal_length: F,
}

fn refract_ray(lens: &IdealLens, ray: &Ray) -> Option<Ray> {
    // Calculate the intersection point between the ray and the lens
    let intersection = ray.intersect(&lens.plane)?;


    // Calculate refracted direction in optical axis of the lens

    let tangent = (intersection - lens.plane.origin).normalize();
    let distance_from_center = (intersection - lens.plane.origin).norm();

    let sin_alpha = tangent.dot(&ray.direction.normalize());
    let alpha = sin_alpha; // Paraxial approximation
    let alpha_out = alpha - distance_from_center/lens.focal_length; // Paraxial approximation

    let normal = lens.plane.normal;
    let cotangent = normal.cross(&tangent);
    let direction_out = na::UnitQuaternion::from_axis_angle(&na::Unit::new_normalize(cotangent), alpha_out) * normal;


    // Return the outgoing ray
    return Some(Ray {
        origin: intersection,
        direction: direction_out,
    });
}

pub struct SphericalSurface {
    pub plane: Plane,
    pub radius: F,
    pub relative_refractive_index: F,
}

impl SphericalSurface {
    pub fn tangent_plane(&self, position: &Point) -> Plane {
        let center = self.plane.origin + self.plane.normal.into_inner() * self.radius;
        let normal = UnitVector::new_normalize(center - position);
        let normal = if self.radius > 0.0 { normal } else { -normal };
        let sphere_pos = center - normal.into_inner() * self.radius;
        Plane {
            origin: sphere_pos,
            normal: normal,
        }
    }
}

impl OpticalSurface for SphericalSurface {
    fn ray_interaction(&self, ray: &Ray) -> Option<Ray> {
        //Assuming distance plane - sphere is small
        let intersection = ray.intersect(&self.plane)?;
        let tangent_plane = self.tangent_plane(&intersection);
        let outgoing_dir = tangent_plane.refract_vector(&ray.direction, self.relative_refractive_index)?;
        Some(Ray {
            origin: tangent_plane.origin,
            direction: outgoing_dir,
        })
    }
}

pub struct SphericalLens {
    pub surface_in: SphericalSurface,
    pub surface_out: SphericalSurface,
}

impl SphericalLens {
    pub fn new(plane: Plane, radius: F, refractive_index: F, thickness: F) -> Self {
        Self {
            surface_in: SphericalSurface {
                plane: plane.offset(-thickness),
                radius: radius,
                relative_refractive_index: refractive_index,
            },
            surface_out: SphericalSurface {
                plane: plane.offset(thickness),
                radius: -radius,
                relative_refractive_index: 1.0/refractive_index,
            },
        }
    }

    pub fn refract_ray(&self, ray: &Ray) -> Option<Ray> {
        //self.surface_in.refract_ray(ray)
        self.surface_out.ray_interaction(&self.surface_in.ray_interaction(ray)?)
    }
}

struct DiffractionGrating {
    pub plane: Plane,
    pub lines_per_mm: F,
}

pub struct SequentialOpticalSystem {
    pub surfaces: Vec<Box<dyn OpticalSurface>>,
}

impl SequentialOpticalSystem {
    pub fn new() -> Self {
        Self {
            surfaces: Vec::new(),
        }
    }

    pub fn trace_ray(&self, incoming: &Ray) -> RayTrace {
        let mut trace = RayTrace {
            points: vec![incoming.origin],
            direction: incoming.direction,
        };
        let mut ray = incoming.clone();
        for surface in &self.surfaces {
            let new_ray = surface.ray_interaction(&ray);
            if let Some(new_ray) = new_ray {
                trace.points.push(new_ray.origin);
                trace.direction = new_ray.direction;
                ray = new_ray;
            } else {
                break;
            }
        }
        trace
    }
}

pub struct RayTrace {
    pub points: Vec<Point>,
    pub direction: UnitVector,
}

mod tests {
    use super::*;

    #[test]
    fn test_ray_intersection() {
        let ray = Ray {
            origin: Vector::new(1.0, 1.0, 0.0),
            direction: Vector::new(1.0, 0.0, 0.0),
        };

        let plane = Plane {
            origin: Vector::new(2.0, 0.0, 0.0),
            normal: Vector::new(1.0, 0.0, 0.0),
        };

        let intersection = ray.intersect(&plane).unwrap();
        assert_eq!(intersection, Vector::new(2.0, 1.0, 0.0));

        let plane2 = Plane {
            origin: Vector::new(2.0, 0.0, 0.0),
            normal: Vector::new(-1.0, 0.0, 0.0),
        };

        let intersection = ray.intersect(&plane2).unwrap();
        assert_eq!(intersection, Vector::new(2.0, 1.0, 0.0));
    }

    #[test]
    fn test_plane_refraction() {
        let plane = Plane {
            origin: Vector::new(2.0, 0.0, 0.0),
            normal: Vector::new(1.0, 0.0, 0.0),
        };

        let incoming = Vector::new(1.0, 0.0, 0.0);
        assert_eq!(plane.refract_vector(&incoming, 1.5).unwrap(), Vector::new(1.0, 0.0, 0.0));

    }
}
