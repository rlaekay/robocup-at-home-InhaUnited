#include "locator.h"
#include "brain.h"
#include "brain_tree.h"
#include "utils/math.h"
#include "utils/misc.h"
#include "utils/print.h"
#include <random>

#define REGISTER_LOCATOR_BUILDER(Name)                                                                                                                         \
  factory.registerBuilder<Name>(#Name, [brain](const string &name, const NodeConfig &config) { return make_unique<Name>(name, config, brain); });

double gaussianRandom(double mean, double stddev) {
  static std::random_device rd;
  static std::mt19937 gen(rd());
  std::normal_distribution<double> d(mean, stddev);
  return d(gen);
}
void RegisterLocatorNodes(BT::BehaviorTreeFactory &factory, Brain *brain) {
  REGISTER_LOCATOR_BUILDER(SelfLocate);
  REGISTER_LOCATOR_BUILDER(SelfLocateEnterField);
  REGISTER_LOCATOR_BUILDER(SelfLocate1M);
  REGISTER_LOCATOR_BUILDER(SelfLocateBorder);
  REGISTER_LOCATOR_BUILDER(SelfLocate2T);
  REGISTER_LOCATOR_BUILDER(SelfLocateLT);
  REGISTER_LOCATOR_BUILDER(SelfLocatePT);
  REGISTER_LOCATOR_BUILDER(SelfLocate2X);
}

void Locator::calcFieldMarkers(FieldDimensions fd) {

  fieldMarkers.push_back(FieldMarker{'X', 0.0, -fd.circleRadius, 0.0});
  fieldMarkers.push_back(FieldMarker{'X', 0.0, fd.circleRadius, 0.0});

  fieldMarkers.push_back(FieldMarker{'P', fd.length / 2 - fd.penaltyDist, 0.0, 0.0});
  fieldMarkers.push_back(FieldMarker{'P', -fd.length / 2 + fd.penaltyDist, 0.0, 0.0});

  fieldMarkers.push_back(FieldMarker{'T', 0.0, fd.width / 2, 0.0});
  fieldMarkers.push_back(FieldMarker{'T', 0.0, -fd.width / 2, 0.0});

  fieldMarkers.push_back(FieldMarker{'L', (fd.length / 2 - fd.penaltyAreaLength), fd.penaltyAreaWidth / 2, 0.0});
  fieldMarkers.push_back(FieldMarker{'L', (fd.length / 2 - fd.penaltyAreaLength), -fd.penaltyAreaWidth / 2, 0.0});
  fieldMarkers.push_back(FieldMarker{'L', -(fd.length / 2 - fd.penaltyAreaLength), fd.penaltyAreaWidth / 2, 0.0});
  fieldMarkers.push_back(FieldMarker{'L', -(fd.length / 2 - fd.penaltyAreaLength), -fd.penaltyAreaWidth / 2, 0.0});
  fieldMarkers.push_back(FieldMarker{'T', fd.length / 2, fd.penaltyAreaWidth / 2, 0.0});
  fieldMarkers.push_back(FieldMarker{'T', fd.length / 2, -fd.penaltyAreaWidth / 2, 0.0});
  fieldMarkers.push_back(FieldMarker{'T', -fd.length / 2, fd.penaltyAreaWidth / 2, 0.0});
  fieldMarkers.push_back(FieldMarker{'T', -fd.length / 2, -fd.penaltyAreaWidth / 2, 0.0});

  fieldMarkers.push_back(FieldMarker{'L', (fd.length / 2 - fd.goalAreaLength), fd.goalAreaWidth / 2, 0.0});
  fieldMarkers.push_back(FieldMarker{'L', (fd.length / 2 - fd.goalAreaLength), -fd.goalAreaWidth / 2, 0.0});
  fieldMarkers.push_back(FieldMarker{'L', -(fd.length / 2 - fd.goalAreaLength), fd.goalAreaWidth / 2, 0.0});
  fieldMarkers.push_back(FieldMarker{'L', -(fd.length / 2 - fd.goalAreaLength), -fd.goalAreaWidth / 2, 0.0});
  fieldMarkers.push_back(FieldMarker{'T', fd.length / 2, fd.goalAreaWidth / 2, 0.0});
  fieldMarkers.push_back(FieldMarker{'T', fd.length / 2, -fd.goalAreaWidth / 2, 0.0});
  fieldMarkers.push_back(FieldMarker{'T', -fd.length / 2, fd.goalAreaWidth / 2, 0.0});
  fieldMarkers.push_back(FieldMarker{'T', -fd.length / 2, -fd.goalAreaWidth / 2, 0.0});

  fieldMarkers.push_back(FieldMarker{'L', fd.length / 2, fd.width / 2, 0.0});
  fieldMarkers.push_back(FieldMarker{'L', fd.length / 2, -fd.width / 2, 0.0});
  fieldMarkers.push_back(FieldMarker{'L', -fd.length / 2, fd.width / 2, 0.0});
  fieldMarkers.push_back(FieldMarker{'L', -fd.length / 2, -fd.width / 2, 0.0});
}

void Locator::setPFParams(int numParticles, double initMargin, bool ownHalf, double sensorNoise, std::vector<double> alphas, double alphaSlow, double alphaFast,
                          double injectionRatio, double zeroMotionTransThresh, double zeroMotionRotThresh, bool resampleWhenStopped, double clusterDistThr,
                          double clusterThetaThr, double smoothAlpha) {
  pfNumParticles = numParticles;
  pfInitFieldMargin = initMargin;
  pfInitOwnHalfOnly = ownHalf;
  pfSensorNoiseR = sensorNoise;
  if (alphas.size() >= 4) {
    pfAlpha1 = alphas[0];
    pfAlpha2 = alphas[1];
    pfAlpha3 = alphas[2];
    pfAlpha4 = alphas[3];
  }
  alpha_slow = alphaSlow;
  alpha_fast = alphaFast;
  pfInjectionRatio = injectionRatio;
  pfZeroMotionTransThresh = zeroMotionTransThresh;
  pfZeroMotionRotThresh = zeroMotionRotThresh;
  pfResampleWhenStopped = resampleWhenStopped;

  pfClusterDistThr = clusterDistThr;
  pfClusterThetaThr = clusterThetaThr;
  pfSmoothAlpha = smoothAlpha;
}

void Locator::init(FieldDimensions fd, int minMarkerCntParam, double residualToleranceParam, double muOffestParam, bool enableLogParam, string logIPParam) {
  fieldDimensions = fd;
  calcFieldMarkers(fd);
  enableLog = enableLogParam;
  logIP = logIPParam;
  // if (enableLog) {
  // logger = &log;
  // auto connectError = log.connect(logIP);
  // if (connectError.is_err()) prtErr(format("Rerun log connect Error: %s", connectError.description.c_str()));
  // auto saveError = log.save("/home/booster/log.rrd");
  // if (saveError.is_err()) prtErr(format("Rerun log save Error: %s", saveError.description.c_str()));
  // }
}

void Locator::globalInitPF(Pose2D currentOdom) {
  double xMin = -fieldDimensions.length / 2.0 - pfInitFieldMargin;
  double xMax = pfInitOwnHalfOnly ? pfInitFieldMargin : (fieldDimensions.length / 2.0 + pfInitFieldMargin);
  double yMin = -fieldDimensions.width / 2.0 - pfInitFieldMargin;
  double yMax = fieldDimensions.width / 2.0 + pfInitFieldMargin;
  double thetaMin = -M_PI;
  double thetaMax = M_PI;

  isPFInitialized = true;
  lastPFOdomPose = currentOdom;

  int num = pfNumParticles;
  pfParticles.resize(num);

  std::srand(std::time(0));
  for (int i = 0; i < num; i++) {
    pfParticles[i].x = xMin + ((double)rand() / RAND_MAX) * (xMax - xMin);
    pfParticles[i].y = yMin + ((double)rand() / RAND_MAX) * (yMax - yMin);
    double thetaSpread = deg2rad(30.0);
    double thetaCenter;

    if (pfParticles[i].y > 0)
      thetaCenter = -M_PI / 2.0;
    else
      thetaCenter = M_PI / 2.0;

    pfParticles[i].theta = toPInPI(thetaCenter + ((double)rand() / RAND_MAX * 2.0 * thetaSpread) - thetaSpread);
    pfParticles[i].weight = 1.0 / num;
  }

  w_slow = 0.0;
  w_fast = 0.0;

  hasSmoothedPose = false;
}

void Locator::predictPF(Pose2D currentOdomPose) {

  prtWarn(format("[PF][predictPF] enter | initialized=%d | pfN=%zu | "
                 "odom=(%.2f %.2f %.2f)",
                 isPFInitialized, pfParticles.size(), currentOdomPose.x, currentOdomPose.y, rad2deg(currentOdomPose.theta)));

  if (!isPFInitialized) {
    prtWarn("[PF][predictPF] NOT initialized -> only update lastPFOdomPose");
    lastPFOdomPose = currentOdomPose; // (0,0,0)에서 점프 방지
    return;
  }

  double dx = currentOdomPose.x - lastPFOdomPose.x;
  double dy = currentOdomPose.y - lastPFOdomPose.y;
  double dtheta = toPInPI(currentOdomPose.theta - lastPFOdomPose.theta);

  // Zero Motion Gate
  double transDist = sqrt(dx * dx + dy * dy);
  double rotDist = fabs(dtheta);

  if (transDist < pfZeroMotionTransThresh && rotDist < pfZeroMotionRotThresh) {
    isRobotMoving = false;
    return;
  }
  isRobotMoving = true;

  double c = cos(lastPFOdomPose.theta);
  double s = sin(lastPFOdomPose.theta);
  double trans_x = c * dx + s * dy; // 로봇좌표계로
  double trans_y = -s * dx + c * dy;
  double rot1 = atan2(trans_y, trans_x);
  double trans = sqrt(trans_x * trans_x + trans_y * trans_y);
  double rot2 = (dtheta - rot1);

  double alpha1 = pfAlpha1;
  double alpha2 = pfAlpha2;
  double alpha3 = pfAlpha3;
  double alpha4 = pfAlpha4;

  for (auto &p : pfParticles) {
    double n_rot1 = rot1 + (gaussianRandom(0, alpha1d * fabs(rot1) + alpha2 * trans));
    double n_trans = trans + (gaussianRandom(0, alpha3 * trans + alpha4 * (fabs(rot1) + fabs(rot2))));
    double n_rot2 = rot2 + (gaussianRandom(0, alpha1 * fabs(rot2) + alpha2 * trans));

    p.x += n_trans * cos(p.theta + n_rot1);
    p.y += n_trans * sin(p.theta + n_rot1);
    p.theta = toPInPI(p.theta + n_rot1 + n_rot2);
  }

  lastPFOdomPose = currentOdomPose;
}

void Locator::correctPF(const vector<FieldMarker> markers) {
  prtWarn(format("[PF][correctPF] enter | initialized=%d | pfN=%zu | markersN=%zu", isPFInitialized, pfParticles.size(), markers.size()));
  if (!isPFInitialized || markers.empty()) {
    prtWarn("[PF][correctPF] not initialized");
    return;
  }

  double sigma = pfSensorNoiseR;
  double totalWeight = 0;
  double avgWeight = 0;

  // Weight Update
  for (auto &p : pfParticles) {
    // Check Boundary Constraints
    double xMinConstraint = -fieldDimensions.length / 2.0 - pfInitFieldMargin;
    double xMaxConstraint = fieldDimensions.length / 2.0 + pfInitFieldMargin;
    double yMinConstraint = -fieldDimensions.width / 2.0 - pfInitFieldMargin;
    double yMaxConstraint = fieldDimensions.width / 2.0 + pfInitFieldMargin;

    if (p.x < xMinConstraint || p.x > xMaxConstraint || p.y < yMinConstraint || p.y > yMaxConstraint) {
      p.weight = 0.0;
    } else {
      Pose2D pose{p.x, p.y, p.theta};
      double logLikelihood = 0;
      for (auto &m_r : markers) {
        auto m_f = markerToFieldFrame(m_r, pose);
        double dist = minDist(m_f);
        logLikelihood += -(dist * dist) / (2 * sigma * sigma);
      }
      double likelihood = exp(logLikelihood);
      p.weight *= likelihood;
    }
    totalWeight += p.weight;
  }

  if (pfParticles.size() > 0) avgWeight = totalWeight / pfParticles.size();

  // Normalize
  if (totalWeight < 1e-10) {
    for (auto &p : pfParticles)
      p.weight = 1.0 / pfParticles.size();
  } else {
    for (auto &p : pfParticles)
      p.weight /= totalWeight;
  }

  // --- Augmented MCL: Update w_slow and w_fast ---
  // DISABLED by user request; favoring pure localization from good init
  // To re-enable, uncomment the injection logic below.
  double p_inject = 0.1;
  /*
  if (w_slow == 0.0)
    w_slow = avgWeight;
  else
    w_slow += alpha_slow * (avgWeight - w_slow);
  if (w_fast == 0.0)
    w_fast = avgWeight;
  else
    w_fast += alpha_fast * (avgWeight - w_fast);

  // Calculate injection probability
  // Calculate injection probability
  double w_diff = 1.0 - w_fast / w_slow;
  p_inject = std::min(pfInjectionRatio, std::max(0.0, w_diff));
  */

  if (isRobotMoving || pfResampleWhenStopped) {
    double sqSum = 0;
    for (auto &p : pfParticles)
      sqSum += p.weight * p.weight;
    double ess = 1.0 / (sqSum + 1e-9);

    if (ess < pfParticles.size() * 0.3) {
      vector<Particle> newParticles;
      newParticles.reserve(pfParticles.size());
      int M = pfParticles.size();
      double r = ((double)rand() / RAND_MAX) * (1.0 / M);
      double c = pfParticles[0].weight;
      int i = 0;

      // for random injection
      double xMin = -fieldDimensions.length / 2.0 - pfInitFieldMargin;
      double xMax = pfInitOwnHalfOnly ? 1.0 : (fieldDimensions.length / 2.0 + pfInitFieldMargin);
      double yMin = -fieldDimensions.width / 2.0 - pfInitFieldMargin;
      double yMax = fieldDimensions.width / 2.0 + pfInitFieldMargin;

      for (int m = 0; m < M; m++) {

        if (((double)rand() / RAND_MAX) < p_inject) {
          Particle newP;
          newP.x = xMin + ((double)rand() / RAND_MAX) * (xMax - xMin);
          newP.y = yMin + ((double)rand() / RAND_MAX) * (yMax - yMin);
          newP.theta = toPInPI(-M_PI + ((double)rand() / RAND_MAX) * 2 * M_PI);
          newP.weight = 1.0 / M;
          newParticles.push_back(newP);
        } else {
          // Low Variance Sampling
          double u = r + (double)m / M;
          while (u > c && i < M - 1) {
            i++;
            c += pfParticles[i].weight;
          }
          Particle newP = pfParticles[i];
          newP.weight = 1.0 / M;
          newParticles.push_back(newP);
        }
      }
      pfParticles = newParticles;
    }
  }
}

Pose2D Locator::getEstimatePF() {
  if (pfParticles.empty()) return {0, 0, 0};

  struct Cluster {
    double totalWeight = 0;
    double xSum = 0;
    double ySum = 0;
    double cosSum = 0;
    double sinSum = 0;
    double leaderX = 0;
    double leaderY = 0;
    double leaderTheta = 0;
  };

  std::vector<Cluster> clusters;

  // Sort
  std::vector<int> sortedIndices(pfParticles.size());
  std::iota(sortedIndices.begin(), sortedIndices.end(), 0);
  std::sort(sortedIndices.begin(), sortedIndices.end(), [&](int a, int b) { return pfParticles[a].weight > pfParticles[b].weight; });

  // clustering
  for (int idx : sortedIndices) {
    auto &p = pfParticles[idx];
    bool added = false;
    for (auto &c : clusters) {
      // 게이팅
      double d = std::hypot(p.x - c.leaderX, p.y - c.leaderY);
      double dTheta = std::fabs(toPInPI(p.theta - c.leaderTheta));
      // weighted sum 구하기
      if (d < pfClusterDistThr && dTheta < pfClusterThetaThr) {
        c.totalWeight += p.weight;
        c.xSum += p.x * p.weight;
        c.ySum += p.y * p.weight;
        c.cosSum += cos(p.theta) * p.weight;
        c.sinSum += sin(p.theta) * p.weight;
        added = true;
        break;
      }
    }
    // cluster에 포함되지 않았다면 다른 클러스터의 대장이 됨
    if (!added) {
      Cluster c;
      c.totalWeight = p.weight;
      c.xSum = p.x * p.weight;
      c.ySum = p.y * p.weight;
      c.cosSum = cos(p.theta) * p.weight;
      c.sinSum += sin(p.theta) * p.weight;
      c.leaderX = p.x;
      c.leaderY = p.y;
      c.leaderTheta = p.theta;
      clusters.push_back(c);
    }
  }

  // 가장 큰 가중치 합을 가진 클러스터 선택
  int bestClusterIdx = -1;
  double maxWeight = -1.0;

  for (int i = 0; i < clusters.size(); i++) {
    if (clusters[i].totalWeight > maxWeight) {
      maxWeight = clusters[i].totalWeight;
      bestClusterIdx = i;
    }
  }

  if (bestClusterIdx == -1) return {0, 0, 0};

  // expected value
  Pose2D rawEstPose;
  auto &bestC = clusters[bestClusterIdx];
  if (bestC.totalWeight > 0) {
    rawEstPose = Pose2D{bestC.xSum / bestC.totalWeight, bestC.ySum / bestC.totalWeight, atan2(bestC.sinSum, bestC.cosSum)}; // 기댓값
  } else {
    rawEstPose = Pose2D{bestC.leaderX, bestC.leaderY, bestC.leaderTheta};
  }

  // EMA smoothing
  if (!hasSmoothedPose) {
    smoothedPose = rawEstPose;
    hasSmoothedPose = true;
  } else {
    smoothedPose.x = pfSmoothAlpha * rawEstPose.x + (1.0 - pfSmoothAlpha) * smoothedPose.x;
    smoothedPose.y = pfSmoothAlpha * rawEstPose.y + (1.0 - pfSmoothAlpha) * smoothedPose.y;
    double diffTheta = toPInPI(rawEstPose.theta - smoothedPose.theta);
    smoothedPose.theta = toPInPI(smoothedPose.theta + pfSmoothAlpha * diffTheta);
  }

  return smoothedPose;
}

void Locator::setLog(rerun::RecordingStream *stream) { logger = stream; }

void Locator::logParticles(double time_sec) {
  if (!enableLog || logger == nullptr) return;

  const size_t pfN = pfParticles.size();

  prtWarn(format("[PF][logParticles] pfN=%zu enableLog=%d", pfParticles.size(), enableLog ? 1 : 0));

  std::vector<std::vector<rerun::Position2D>> lines;
  lines.reserve(pfN);

  const float len = 0.1f;

  for (const auto &p : pfParticles) {
    float x0 = static_cast<float>(p.x);
    float y0 = static_cast<float>(p.y);
    float x1 = x0 + len * std::cos(p.theta);
    float y1 = y0 + len * std::sin(p.theta);

    lines.push_back({{x0, -y0}, {x1, -y1}});
  }

  std::vector<rerun::Color> colors(pfN, rerun::Color{0, 255, 255, 120});

  // 얇은 선
  std::vector<float> radii(pfN, 0.0025f);

  logger->log("field/particles", rerun::LineStrips2D(lines).with_colors(colors).with_radii(radii).with_draw_order(19.0));
}

FieldMarker Locator::markerToFieldFrame(FieldMarker marker_r, Pose2D pose_r2f) {
  auto [x, y, theta] = pose_r2f;

  Eigen::Matrix3d transform;
  transform << cos(theta), -sin(theta), x, sin(theta), cos(theta), y, 0, 0, 1;

  Eigen::Vector3d point_r;
  point_r << marker_r.x, marker_r.y, 1.0;

  auto point_f = transform * point_r;

  return FieldMarker{marker_r.type, point_f.x(), point_f.y(), marker_r.confidence};
}

double Locator::minDist(FieldMarker marker) {
  double minDist = std::numeric_limits<double>::infinity();
  double dist;
  for (int i = 0; i < fieldMarkers.size(); i++) {
    auto target = fieldMarkers[i];
    if (target.type != marker.type) { continue; }
    dist = sqrt(pow((target.x - marker.x), 2.0) + pow((target.y - marker.y), 2.0));
    if (dist < minDist) minDist = dist;
  }
  return minDist;
}

// 나중에 모드를 설정해서 initial particle 영역을 달리해야댐
NodeStatus SelfLocateEnterField::tick() {
  if (!brain->locator->getIsPFInitialized()) { brain->locator->globalInitPF(brain->data->robotPoseToOdom); }
  return NodeStatus::SUCCESS;
}

NodeStatus SelfLocate::tick() { return NodeStatus::SUCCESS; }

NodeStatus SelfLocate1M::tick() { return NodeStatus::SUCCESS; }

NodeStatus SelfLocate2X::tick() { return NodeStatus::SUCCESS; }

NodeStatus SelfLocate2T::tick() { return NodeStatus::SUCCESS; }

NodeStatus SelfLocateLT::tick() { return NodeStatus::SUCCESS; }

NodeStatus SelfLocatePT::tick() { return NodeStatus::SUCCESS; }

NodeStatus SelfLocateBorder::tick() { return NodeStatus::SUCCESS; }