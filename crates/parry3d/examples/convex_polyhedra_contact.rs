use nalgebra::Point3;
use parry3d::math::Isometry;
use parry3d::query;
use parry3d::query::PointQuery;
use parry3d::shape::ConvexPolyhedron;

// Issue #157.
fn main() {
    let convex_polyhedron_a_points = [
        [-0.7938391, 5.1101756, 0.1773476],
        [-5.293839, 0.6101756, -4.3226523],
        [-5.293839, 0.6101756, 5.6773477],
        [-0.7938391, 5.1101756, 1.1773477],
        [-0.7938391, 6.1101756, 0.1773476],
        [-5.293839, 10.610176, -4.3226523], // this point and only this point lies inside of b
        [-5.293839, 10.610176, 5.6773477],
        [-0.7938391, 6.1101756, 1.1773477],
    ]
    .map(|arr| Point3::new(arr[0], arr[1], arr[2]));
    let convex_polyhedron_a =
        ConvexPolyhedron::from_convex_hull(&convex_polyhedron_a_points).unwrap();

    let convex_polyhedron_b_points = [
        [8.114634, 4.6308937, 0.76987207],
        [-18.83664, -53.96347, -649.29565],
        [-608.7832, -53.96347, -208.59384],
        [6.93474, 4.6308937, 1.6512758],
        [8.253552, 5.426136, 0.9558364],
        [50.62288, 343.65768, -556.3136],
        [-539.3237, 343.65768, -115.611725],
        [7.073659, 5.426136, 1.8372401],
    ]
    .map(|arr| Point3::new(arr[0], arr[1], arr[2]));
    let convex_polyhedron_b =
        ConvexPolyhedron::from_convex_hull(&convex_polyhedron_b_points).unwrap();

    let num_contained_points = convex_polyhedron_a_points
        .iter()
        .filter(|point| {
            convex_polyhedron_b
                .project_local_point(&point, false)
                .is_inside
        })
        .count();

    let contact = query::contact(
        &Isometry::identity(),
        &convex_polyhedron_a,
        &Isometry::identity(),
        &convex_polyhedron_b,
        0.0,
    )
    .unwrap();

    assert!(num_contained_points == 1);
    assert!(contact.is_some());
}
