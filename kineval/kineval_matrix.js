//////////////////////////////////////////////////
/////     MATRIX ALGEBRA AND GEOMETRIC TRANSFORMS 
//////////////////////////////////////////////////

function matrix_copy(m1) {
    // returns 2D array that is a copy of m1

    var mat = [];
    var i, j;

    for (i = 0; i < m1.length; i++) { // for each row of m1
        mat[i] = [];
        for (j = 0; j < m1[0].length; j++) { // for each column of m1
            mat[i][j] = m1[i][j];
        }
    }
    return mat;
}


// STENCIL: reference matrix code has the following functions:
//   matrix_multiply
//   matrix_transpose
//   matrix_pseudoinverse
//   matrix_invert_affine
//   vector_normalize
//   vector_cross
//   generate_identity
//   generate_translation_matrix
//   generate_rotation_matrix_X
//   generate_rotation_matrix_Y
//   generate_rotation_matrix_Z



// **** Function stencils are provided below, please uncomment and implement them ****//



function matrix_multiply(m1, m2) {
    // returns 2D array that is the result of m1*m2
    var ans = new Array(m1.length);
    for (var i = 0; i < ans.length; i++) {
        ans[i] = new Array(m2[0].length);
    }
    for (var i = 0; i < m1.length; i++) {
        for (var j = 0; j < m2[0].length; j++) {
            ans[i][j] = 0;
            for (var k = 0; k < m1[0].length; k++) {
                ans[i][j] += m1[i][k] * m2[k][j];
            }
        }
    }

    return ans;

}

function matrix_transpose(m) {
    // returns 2D array that is the result of m1*m2
    var ans = matrix_copy(m);
    for (var i = 0; i < m.length; i++) {
        for (var j = 0; j < m[0].length; j++) {
            ans[j][i] = m[i][j];
        }
    }
    return ans;
}

// function matrix_pseudoinverse(m) {
//     // returns pseudoinverse of matrix m

// }

// function matrix_invert_affine(m) {
//     // returns 2D array that is the invert affine of 4-by-4 matrix m

// }

function vector_normalize(v) {
    // returns normalized vector for v
    var total = 0;
    var ans = new Array(v.length);
    for (var i = 0; i < v.length; i++) {
        total += (v[i] * v[i]);
    }
    var length = Math.sqrt(total);

    for (var i = 0; i < ans.length; i++) {
        ans[i] = v[i] / length;
    }

    return ans;

}

function vector_cross(a, b) {
    // return cross product of vector a and b with both has 3 dimensions
    var i = a[1] * b[2] - a[2] * b[1];
    var j = a[0] * b[2] - a[2] * b[0];
    var k = a[0] * b[1] - a[1] * b[0];

    var ans = new Array(3);

    ans[0] = i;
    ans[1] = -1 * j;
    ans[2] = k;

    return ans;

}

function generate_identity() {
    // returns 4-by-4 2D array of identity matrix
    var ans = new Array(4);
    for (var i = 0; i < ans.length; i++) {
        ans[i] = new Array(4).fill(0);
    }

    ans[0][0] = 1;
    ans[1][1] = 1;
    ans[2][2] = 1;
    ans[3][3] = 1;

    return ans;

}

function generate_translation_matrix(tx, ty, tz) {
    // returns 4-by-4 matrix as a 2D array
    var ans = generate_identity();
    ans[0][3] = tx;
    ans[1][3] = ty;
    ans[2][3] = tz;
    return ans;

}

function generate_rotation_matrix_X(angle) {
    // returns 4-by-4 matrix as a 2D array, angle is in radians
    var ans = generate_identity();
    ans[1][1] = Math.cos(angle);
    ans[1][2] = -1 * Math.sin(angle);
    ans[2][2] = Math.cos(angle);
    ans[2][1] = Math.sin(angle);

    return ans;
}

function generate_rotation_matrix_Y(angle) {
    // returns 4-by-4 matrix as a 2D array, angle is in radians
    var ans = generate_identity();
    ans[0][0] = Math.cos(angle);
    ans[0][2] = Math.sin(angle);
    ans[2][0] = -1 * Math.sin(angle);
    ans[2][2] = Math.cos(angle);

    return ans;
}

function generate_rotation_matrix_Z(angle) {
    // returns 4-by-4 matrix as a 2D array, angle is in radians
    var ans = generate_identity();
    ans[0][0] = Math.cos(angle);
    ans[1][0] = Math.sin(angle);
    ans[0][1] = -1 * Math.sin(angle);
    ans[1][1] = Math.cos(angle);

    return ans;
}