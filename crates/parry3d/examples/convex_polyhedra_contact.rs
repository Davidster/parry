use nalgebra::Point3;
use parry3d::math::Isometry;
use parry3d::query;
use parry3d::query::PointQuery;
use parry3d::shape::ConvexPolyhedron;

// Issue #157.
fn main() {
    let convex_polyhedron_a_points = [
        [0.45838028, 5.7372417, 0.61019015],
        [1000.35846, -994.1628, 1000.51013],
        [1000.35834, -994.1628, -999.48987],
        [0.45838028, 5.7372417, 0.41019014],
        [0.45838028, 5.9372416, 0.61019015],
        [1000.35846, 1005.8372, 1000.51013],
        [1000.35834, 1005.8372, -999.48987],
        [0.45838028, 5.9372416, 0.41019014],
    ]
    .map(|arr| Point3::new(arr[0], arr[1], arr[2]));
    let convex_polyhedron_a =
        ConvexPolyhedron::from_convex_hull(&convex_polyhedron_a_points).unwrap();

    let convex_polyhedron_b_points = [
        [-0.4522132, 8.780189, 15.508082],
        [72537.12, -72390.1, -81439.88],
        [-74725.21, -72390.1, -79438.195],
        [-0.45368585, 8.780189, 15.508101],
        [-0.45221695, 8.780969, 15.507806],
        [72161.625, 5710.25, -109064.305],
        [-75100.7, 5710.25, -107062.62],
        [-0.4536896, 8.780969, 15.507825],
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

    let intersects = query::intersection_test(
        &Isometry::identity(),
        &convex_polyhedron_a,
        &Isometry::identity(),
        &convex_polyhedron_b,
    )
    .unwrap();

    assert!(num_contained_points == 4);
    assert!(intersects);
}
