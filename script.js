function vecAdd(a, b) {
	return [
  	a[0] + b[0],
    a[1] + b[1],
  ];
}

function matAdd(a, b) {
	return [
  	[a[0][0] + b[0][0], a[0][1] + b[0][1]],
    [a[1][0] + b[1][0], a[1][1] + b[1][1]],
  ];
}

function vecScale(a, s) {
  return [
    a[0]*s,
    a[1]*s,
  ];
}

function matScale(a, s) {
  return [
    [a[0][0]*s, a[0][1]*s],
    [a[1][0]*s, a[1][1]*s],
  ];
}

function matMul(a, b) {
  return [
    [a[0][0]*b[0][0] + a[0][1]*b[1][0], a[0][0]*b[0][1] + a[0][1]*b[1][1]],
    [a[1][0]*b[0][0] + a[1][1]*b[1][0], a[1][0]*b[0][1] + a[1][1]*b[1][1]],
  ];
}

function matVecMul(a, b) {
  return [
    a[0][0]*b[0] + a[0][1]*b[1],
    a[1][0]*b[0] + a[1][1]*b[1],
  ];
}

function matInv(a) {
	return matScale(
  	[
    	[a[1][1], -a[0][1]],
      [-a[1][0], a[0][0]],
    ],
    1.0 / (a[0][0]*a[1][1] - a[0][1]*a[1][0]),
  );
}

function matT(a) {
  return [
    [a[0][0], a[1][0]],
    [a[0][1], a[1][1]],
  ];
}

function vecNorm(a) {
	return Math.hypot(a[0], a[1]);
}

function vecAngle(a) {
	return Math.atan2(a[1], a[0]);
}

function rotationMatrix(angle) {
  const c = Math.cos(angle);
  const s = Math.sin(angle);
  return [
    [c, -s],
    [s, c],
  ];
}

function cholesky(m) {
	// http://metamerist.blogspot.com/2008/03/googlaziness-cholesky-2x2.html
  // Find L = [[a, 0], [b, c]], such that M = L * L^T.
  // Expanding that: M = L * L^T = [[a^2, a*b], [a*b, c^2 + b^2]]
  
  // Assumes that m is symmetric and diagonal elements are non-zero.
  const a = Math.sqrt(m[0][0]);
  const b = m[0][1] / a;
  const c = Math.sqrt(m[1][1] - b*b);
  return [[a, 0], [b, c]];
}

function sampleNormalDistribution(mean, cov) {
	// https://en.wikipedia.org/wiki/Multivariate_normal_distribution#Drawing_values_from_the_distribution
  const A = cholesky(cov);
  
  // https://en.wikipedia.org/wiki/Box-Muller_transform
  const r = Math.sqrt(-2 * Math.log(Math.random()+1e-6));
  const s = 2 * Math.PI * Math.random();
  const z = [r * Math.cos(s), r * Math.sin(s)];
  
  return vecAdd(mean, matVecMul(A, z));
}

function setErrorEllipse(element, center, cov, p) {
	// https://www.xarg.org/2018/04/how-to-plot-a-covariance-error-ellipse/

  p = p || 0.95;

  const s = -2 * Math.log(1 - p);

  const a = cov[0][0];
  const b = cov[0][1];
  const c = cov[1][0];
  const d = cov[1][1];
  
  let rx, ry, rotation;

  if (Math.abs(c) < 1e-6) {
  	rx = Math.sqrt(s * a);
  	ry = Math.sqrt(s * d);
    rotation = 0;
  } else {
    const tmp = Math.sqrt((a - d) * (a - d) + 4 * b * c);
    let V = [
      [-(tmp - a + d) / (2 * c), (tmp + a - d) / (2 * c)],
      [1, 1]
    ];
    const sqrtD = [
      Math.sqrt(s * (a + d - tmp) / 2),
      Math.sqrt(s * (a + d + tmp) / 2)
    ];

    const norm1 = Math.hypot(V[0][0], 1);
    const norm2 = Math.hypot(V[0][1], 1);
    V[0][0] /= norm1;
    V[1][0] /= norm1;
    V[0][1] /= norm2;
    V[1][1] /= norm2;

    const ndx = sqrtD[0] < sqrtD[1] ? 1 : 0;

    const x1 = V[0][ndx] * sqrtD[ndx];
    const y1 = V[1][ndx] * sqrtD[ndx];

    const x2 = V[0][1 - ndx] * sqrtD[1 - ndx];
    const y2 = V[1][1 - ndx] * sqrtD[1 - ndx];

    rx = Math.hypot(x1, y1);
    ry = Math.hypot(x2, y2);
    rotation = Math.atan2(y1, x1);
  }

  element.setAttribute("cx", center[0]);
  element.setAttribute("cy", center[1]);
  element.setAttribute("rx", rx);
  element.setAttribute("ry", ry);
  element.setAttribute("transform", `rotate(${rotation * 180 / Math.PI})`);
}

function setRobotRect(element, center) {
  const size = 1;
  element.setAttribute("x", center[0] - size / 2.0);
  element.setAttribute("y", center[1] - size / 2.0);
  element.setAttribute("width", size);
  element.setAttribute("height", size);
}

const rect_robot = document.getElementById("robot");
const ellipse_cov_est = document.getElementById("cov_est");
const ellipse_cov_landmark = document.getElementById("cov_landmark");
const move_distance = document.getElementById("move_distance");
const move_var_track = document.getElementById("move_var_track");
const move_var_cross = document.getElementById("move_var_cross");
const move_error = document.getElementById("move_error");
const landmark_var_x = document.getElementById("landmark_var_x");
const landmark_var_y = document.getElementById("landmark_var_y");
const landmark_cov_xy = document.getElementById("landmark_cov_xy");
const landmark_sym_cov_xy = document.getElementById("landmark_sym_cov_xy");
const landmark_error = document.getElementById("landmark_error");
const landmark_update = document.getElementById("landmark_update");
const landmark_update_help = document.getElementById("landmark_update_help");
const actual_pos_x = document.getElementById("actual_pos_x");
const actual_pos_y = document.getElementById("actual_pos_y");
const est_pos_x = document.getElementById("est_pos_x");
const est_pos_y = document.getElementById("est_pos_y");
const est_pos_var_x = document.getElementById("est_pos_var_x");
const est_pos_var_y = document.getElementById("est_pos_var_y");
const est_pos_cov_xy = document.getElementById("est_pos_cov_xy");
const est_pos_sym_cov_xy = document.getElementById("est_pos_sym_cov_xy");

var pos_actual;
var pos_est;
var cov_est;
var pos_landmark;

function resetRobot() {
	const initial_stddev = 0.1;

	pos_actual = [0, 0];
  pos_est = [0, 0];
  cov_est = [[initial_stddev*initial_stddev, 0], [0, initial_stddev*initial_stddev]];
  
  pos_landmark = null;
  
  displayUpdate();
}

function getLandmarkCovariance() {
  return [
    [+landmark_var_x.value, +landmark_cov_xy.value],
    [+landmark_cov_xy.value, +landmark_var_y.value]
  ];
}

function displayUpdate() {
  landmark_sym_cov_xy.value = landmark_cov_xy.value;
  
  actual_pos_x.value = pos_actual[0].toFixed(3);
  actual_pos_y.value = pos_actual[1].toFixed(3);
  est_pos_x.value = pos_est[0].toFixed(3);
  est_pos_y.value = pos_est[1].toFixed(3);
  est_pos_var_x.value = cov_est[0][0].toFixed(3);
  est_pos_var_y.value = cov_est[1][1].toFixed(3);
  est_pos_cov_xy.value = cov_est[0][1].toFixed(3);
  est_pos_sym_cov_xy.value = cov_est[1][0].toFixed(3);

  const cov_landmark = getLandmarkCovariance();

  setErrorEllipse(ellipse_cov_est, pos_est, cov_est);
  
  if (pos_landmark !== null) {
    ellipse_cov_landmark.style.display = "block";
    setErrorEllipse(ellipse_cov_landmark, pos_landmark, cov_landmark);

    landmark_update.disabled = false;
    landmark_update_help.style.display = "block";
  } else {
    ellipse_cov_landmark.style.display = "none";
    
    landmark_update.disabled = true;
    landmark_update_help.style.display = "none";
  }
  
  setRobotRect(rect_robot, pos_actual);
}

function moveRobot(delta) {
	const distance = +move_distance.value;
	if (distance == 0) {
	  return;
  }

  delta = vecScale(delta, distance / vecNorm(delta));
  
  const cov_unit = [[+move_var_track.value, 0], [0, +move_var_cross.value]];
  
  const angle = vecAngle(delta);
  
  const heading = rotationMatrix(angle);
  
  const cov_delta = matMul(matMul(heading, matScale(cov_unit, distance)), matT(heading));
  
  cov_est = matAdd(cov_est, cov_delta);
  
  pos_est = vecAdd(pos_est, delta);
  
  pos_actual = vecAdd(pos_actual, delta);
  if (move_error.checked) {
  	pos_actual = sampleNormalDistribution(pos_actual, cov_delta);
  }
  
  
  pos_landmark = null;
  
  displayUpdate();
}

function landmarkMeasure() {
	pos_landmark = pos_actual;
  
  if (landmark_error.checked) {
  	pos_landmark = sampleNormalDistribution(pos_landmark, getLandmarkCovariance());
  }

	displayUpdate();
}

function landmarkUpdate() {
  const cov_landmark = getLandmarkCovariance();

	const norm = matInv(matAdd(cov_landmark, cov_est));
  const est_factor = matMul(cov_landmark, norm);
  pos_est = vecAdd(
  	matVecMul(est_factor, pos_est),
    matVecMul(matMul(cov_est, norm), pos_landmark),
  );
  cov_est = matMul(est_factor, cov_est);
  
  pos_landmark = null;
  
  displayUpdate();
}

landmark_var_x.addEventListener("input", displayUpdate);
landmark_var_y.addEventListener("input", displayUpdate);
landmark_cov_xy.addEventListener("input", displayUpdate);

resetRobot();
displayUpdate();
