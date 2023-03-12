//////////////////////////////////////////////////
/////     QUATERNION TRANSFORM ROUTINES 
//////////////////////////////////////////////////

// STENCIL: reference quaternion code has the following functions:
//   quaternion_from_axisangle
//   quaternion_normalize
//   quaternion_to_rotation_matrix
//   quaternion_multiply

// **** Function stencils are provided below, please uncomment and implement them ****//

kineval.quaternionFromAxisAngle = function quaternion_from_axisangle(axis, angle) {
    // returns quaternion q as dic, with q.a as real number, q.b as i component, q.c as j component, q.d as k component
    var q = {};
    q.a = Math.cos(angle / 2);
    q.b = axis[0] * Math.sin(angle / 2);
    q.c = axis[1] * Math.sin(angle / 2);
    q.d = axis[2] * Math.sin(angle / 2);
    return q;
}

kineval.quaternionNormalize = function quaternion_normalize(q1) {
    // returns quaternion q as dic, with q.a as real number, q.b as i component, q.c as j component, q.d as k component
    var q = {};
    var magnitude = Math.sqrt(q1.a * q1.a + q1.b * q1.b + q1.c * q1.c + q1.d * q1.d);
    q.a = q1.a / magnitude;
    q.b = q1.b / magnitude;
    q.c = q1.c / magnitude;
    q.d = q1.d / magnitude;

    return q;
}

kineval.quaternionMultiply = function quaternion_multiply(q1, q2) {
    // returns quaternion q as dic, with q.a as real number, q.b as i component, q.c as j component, q.d as k component
    var q = {};
    q.a = q1.a * q2.a - q1.b * q2.b - q1.c * q2.c - q1.d * q2.d;
    q.b = q1.a * q2.b + q1.b * q2.a + q1.c * q2.d - q1.d * q2.c;
    q.c = q1.a * q2.c - q1.b * q2.d + q1.c * q2.a + q1.d * q2.b;
    q.d = q1.a * q2.d + q1.b * q2.c - q1.c * q2.b + q1.d * q2.a;

    return q;
}

kineval.quaternionToRotationMatrix = function quaternion_to_rotation_matrix(q) {
    // returns 4-by-4 2D rotation matrix

    const aa = q.a * q.a, bb = q.b * q.b, cc = q.c * q.c, dd = q.d * q.d;
    const ab = q.a * q.b, ac = q.a * q.c, ad = q.a * q.d;
    const bc = q.b * q.c, bd = q.b * q.d;
    const cd = q.c * q.d;

    var mat = [
        [aa + bb - cc - dd, 2 * (bc - ad), 2 * (ac + bd), 0],
        [2 * (bc + ad), aa - bb + cc - dd, 2 * (cd - ab), 0],
        [2 * (bd - ac), 2 * (ab + cd), aa - bb - cc + dd, 0],
        [0, 0, 0, 1]
    ];

    return mat;


}