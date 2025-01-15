#include <algorithm>
#include <cassert>
#include <cmath>
#include <format>
#include <imgui.h>
#include <Novice.h>

#include "Math/MathLib.h"
#include "Math/Matrix/Mat4.h"

const char kWindowTitle[] = "LE2B_24_ミズサワ_ハミル_MT4_01_03_確認課題";

constexpr int kWindowWidth = 1280;
constexpr int kWindowHeight = 720;

constexpr float deltaTime = 1.0f / 60.0f;

static const int kRowHeight = 20;
static const int kColumnWidth = 60;

static const float kMouseSensitivity = 0.0022f;
static const float kMoveSpeed = 0.1f;

#pragma region Functions
static float Clamp(const float value, const float min, const float max);

#pragma region Structs
struct Line {
	Vec3 origin; //!< 始点
	Vec3 diff; //!< 終点への差分ベクトル
};

struct Ray {
	Vec3 origin; //!< 始点
	Vec3 diff; //!< 終点への差分ベクトル
};

struct Segment {
	Vec3 origin; //!< 始点
	Vec3 diff; //!< 終点への差分ベクトル
};

struct Plane {
	Vec3 normal; //!< 法線
	float distance; //!< 距離
};

struct Sphere {
	Vec3 center;
	float radius;
};

struct AABB {
	Vec3 min; //!< 最小点
	Vec3 max; //!< 最大点
};

struct OBB {
	Vec3 center;
	Vec3 orientations[3];
	Vec3 size;
};

struct Triangle {
	Vec3 vertices[3]; //!< 頂点
};

struct Spring {
	// アンカー。固定された端の位置
	Vec3 anchor;
	float naturalLength; // 自然長
	float stiffness; // 剛性。ばね定数k
	float dampingCoefficient; // 減衰係数
};

struct Ball {
	Vec3 position; // ボールの位置
	Vec3 velocity; // ボールの速度
	Vec3 acceleration; // ボールの加速度
	float mass; // 質量
	float radius; // 半径
	unsigned int color; // 色
};

struct Pendulum {
	Vec3 anchor; // アンカーポイント。固定された端の位置
	float length; // 紐の長さ
	float angle; // 現在の角度
	float angularVelocity; // 角速度ω
	float angularAcceleration; // 角加速度
};

struct ConicalPendulum {
	Vec3 anchor; // アンカーポイント。固定された端の位置
	float length; // 紐の長さ
	float halfApexAngle; // 円錐の頂角の半分
	float angle; // 現在の角度
	float angularVelocity; // 角速度
};
#pragma endregion

static bool IsCollision(const Sphere& sphere1, const Sphere& sphere2);
static bool IsCollision(const Sphere& sphere, const Plane& plane);
static bool IsCollision(const Segment& segment, const Plane& plane);
static bool IsCollision(const Triangle& triangle, const Segment& segment);
static bool IsCollision(const AABB& a, const AABB& b);
static bool IsCollision(const AABB& aabb, const Sphere& sphere);
static bool IsCollision(const AABB& aabb, const Segment& segment);
static bool IsCollision(const OBB& obb, const Sphere& sphere);
static bool IsCollision(const OBB& obb1, const OBB& obb2);

#pragma region Vector3
static Vec3 Lerp(const Vec3& start, const Vec3& end, float t);
static Vec3 Reflect(const Vec3& input, const Vec3& normal);
static Vec3 ClosestPoint(const Vec3& point, const Segment& segment);
static Vec3 Project(const Vec3& v1, const Vec3& v2);
static Vec3 Cubic(Vec3 y0, Vec3 y1, Vec3 y2, Vec3 y3, float t) {
	Vec3 c0 = y1 - y2;
	Vec3 c1 = (y2 - y0) * 0.5;
	Vec3 c2 = c0 + c1;
	Vec3 c3 = c0 + c2 + (y3 - y1) * 0.5;
	return ((c3 * t - c2 - c3) * t + c1) * t + y1;
}

static Vec3 Perpendicular(const Vec3& vector);
#pragma endregion

static void MatrixScreenPrintf(const int x, const int y, const Mat4& matrix, const char* label);
static void VectorScreenPrintf(const int x, const int y, const Vec3& vector, const char* label);
static void QuaternionScreenPrintf(const int x, const int y, const Quaternion& quaternion, const char* label);

static void DrawSphere(
	const Sphere& sphere, const Mat4& viewProjectionMatrix, const Mat4& viewportMatrix, uint32_t color
);
static void DrawGrid(const Mat4& viewProjectionMatrix, const Mat4& viewportMatrix);
static void DrawPlane(const Plane& plane, const Mat4& viewProjectionMatrix, const Mat4& viewportMatrix, uint32_t color);
static void DrawTriangle(
	const Triangle& triangle, const Mat4& viewProjectionMatrix, const Mat4& viewportMatrix, uint32_t color
);
static void DrawAABB(
	const AABB& aabb, const Mat4& viewProjectionMatrix, const Mat4& viewportMatrix, uint32_t color
);
static void DrawSegment(
	const Segment& segment, const Mat4& viewProjectionMatrix, const Mat4& viewportMatrix, unsigned int color
);
static void DrawLine(
	const Mat4& start, const Mat4& end, const Mat4& viewProjectionMatrix, const Mat4& viewportMatrix, unsigned int color
);
static void DrawObb(const OBB& obb, const Mat4& viewProjectionMatrix, const Mat4& viewportMatrix, uint32_t color);
static void DrawBezier(
	const Vec3& controlPoint0, const Vec3& controlPoint1, const Vec3& controlPoint2, const Mat4& viewProjectionMatrix,
	const Mat4& viewportMatrix, uint32_t color
);
static void DrawCatmullRom(
	const Vec3& controlPoint0, const Vec3& controlPoint1, const Vec3& controlPoint2, const Vec3& controlPoint3,
	const Mat4& viewProjectionMatrix, const Mat4& viewportMatrix, uint32_t color
);
#pragma endregion

constexpr Vec3 kGravity = { 0.0f, -9.8f, 0.0f };

// Windowsアプリでのエントリーポイント(main関数)
int WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR, int) {
	// ライブラリの初期化
	Novice::Initialize(kWindowTitle, 1280, 720);

	// キー入力結果を受け取る箱
	char keys[256] = { 0 };
	char preKeys[256] = { 0 };

	// カメラ
	Vec3 cameraTranslate = { 0.0f, 1.9f, -6.49f };
	Vec3 cameraRotate = { 0.26f, 0.0f, 0.0f };
	float cameraFov = 90.0f;
	int mouseX, mouseY;
	Novice::GetMousePosition(&mouseX, &mouseY);

	Quaternion q1 = { 2.0f , 3.0f, 4.0f, 1.0f };
	Quaternion q2 = { 1.0f, 3.0f, 5.0f, 2.0f };
	Quaternion identity = Quaternion::identity;
	Quaternion conj = q1.Conjugate();
	Quaternion inv = q1.Inverse();
	Quaternion normal = q1.Normalized();
	Quaternion mul1 = q1 * q2;
	Quaternion mul2 = q2 * q1;
	float norm = q1.Norm();

	// ウィンドウの×ボタンが押されるまでループ
	while (Novice::ProcessMessage() == 0) {
		// フレームの開始
		Novice::BeginFrame();

		// キー入力を受け取る
		memcpy(preKeys, keys, 256);
		Novice::GetHitKeyStateAll(keys);

		///
		/// ↓更新処理ここから
		///

#pragma region カメラ操作
		// 前のフレームのマウス位置を保存する
		int preMouseX = mouseX;
		int preMouseY = mouseY;

		// 現在のフレームのマウス位置を取得する
		Novice::GetMousePosition(&mouseX, &mouseY);

		// 前のフレームとの差分を計算する
		Vec3 mouseDelta = { static_cast<float>(mouseX - preMouseX), static_cast<float>(mouseY - preMouseY), 0.0f };

		if (Novice::IsPressMouse(1)) {
			cameraRotate = {
				cameraRotate.x + mouseDelta.y * kMouseSensitivity, cameraRotate.y + mouseDelta.x * kMouseSensitivity,
				cameraRotate.z
			};
		}

		cameraRotate = {
			Clamp(cameraRotate.x, -89.9f * Math::deg2Rad, 89.9f * Math::deg2Rad), cameraRotate.y, cameraRotate.z
		};

		// カメラの回転行列を生成する
		Mat4 cameraRotationMatrix = Mat4::Affine(
			{ 1.0f, 1.0f, 1.0f }, cameraRotate,
			{ 0.0f, 0.0f, 0.0f }
		);

		Vec3 cameraForward = {
			cameraRotationMatrix.m[2][0],
			cameraRotationMatrix.m[2][1],
			cameraRotationMatrix.m[2][2]
		};

		Vec3 cameraRight = {
			cameraRotationMatrix.m[0][0],
			cameraRotationMatrix.m[0][1],
			cameraRotationMatrix.m[0][2]
		};

		Vec3 cameraUp = {
			cameraRotationMatrix.m[1][0],
			cameraRotationMatrix.m[1][1],
			cameraRotationMatrix.m[1][2]
		};

		Vec3 moveInput = { 0.0f, 0.0f, 0.0f };

		if (keys[DIK_W] && !keys[DIK_S]) {
			moveInput.z = 1.0f;
		} else if (!keys[DIK_W] && keys[DIK_S]) {
			moveInput.z = -1.0f;
		}

		if (keys[DIK_D] && !keys[DIK_A]) {
			moveInput.x = 1.0f;
		} else if (!keys[DIK_D] && keys[DIK_A]) {
			moveInput.x = -1.0f;
		}

		if (keys[DIK_E] && !keys[DIK_Q]) {
			moveInput.y = 1.0f;
		} else if (!keys[DIK_E] && keys[DIK_Q]) {
			moveInput.y = -1.0f;
		}

		moveInput = moveInput.Normalized();

		cameraTranslate = cameraTranslate + (cameraForward * moveInput.z + cameraRight * moveInput.x + cameraUp *
			moveInput.y) * kMoveSpeed;
#pragma endregion

		///
		/// ↑更新処理ここまで
		///

		///
		/// ↓描画処理ここから
		///

		// 各種行列の計算
		Mat4 worldMatrix = Mat4::Affine({ 1.0f, 1.0f, 1.0f }, { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f });
		Mat4 cameraMatrix = Mat4::Affine({ 1.0f, 1.0f, 1.0f }, cameraRotate, cameraTranslate);
		Mat4 viewMatrix = cameraMatrix.Inverse();
		Mat4 projectionMatrix = Mat4::PerspectiveFovMat(
			cameraFov * Math::deg2Rad, static_cast<float>(kWindowWidth) / static_cast<float>(kWindowHeight), 0.1f,
			100.0f
		);
		Mat4 worldViewProjectionMatrix = worldMatrix * viewMatrix * projectionMatrix;
		Mat4 viewportMatrix = Mat4::ViewportMat(
			0, 0, static_cast<float>(kWindowWidth),
			static_cast<float>(kWindowHeight), 0.0f, 1.0f
		);

		DrawGrid(worldViewProjectionMatrix, viewportMatrix);

		ImGui::Begin("Parameter");
		ImGui::Text("Camera");
		ImGui::TextWrapped("Hold the right-click and drag to rotate the camera.");
		ImGui::TextWrapped("WASD : move");
		ImGui::TextWrapped("E : moveUp");
		ImGui::TextWrapped("Q : moveDown");
		ImGui::DragFloat3("Pos", &cameraTranslate.x, 0.01f);
		ImGui::DragFloat3("Rot", &cameraRotate.x, 0.01f);
		ImGui::DragFloat("Fov(deg)", &cameraFov, 0.1f);

		ImGui::End();

		QuaternionScreenPrintf(0, 0, identity, "Identity");
		QuaternionScreenPrintf(0, kRowHeight, conj, "Conjugate");
		QuaternionScreenPrintf(0, kRowHeight * 2, inv, "Inverse");
		QuaternionScreenPrintf(0, kRowHeight * 3, normal, "Normalize");
		QuaternionScreenPrintf(0, kRowHeight * 4, mul1, "Multiply(q1, q2)");
		QuaternionScreenPrintf(0, kRowHeight * 5, mul2, "Multiply(q2, q1)");
		Novice::ScreenPrintf(0, kRowHeight * 6, "%.2f		: Norm", norm);

		///
		/// ↑描画処理ここまで
		///

		// フレームの終了
		Novice::EndFrame();

		// ESCキーが押されたらループを抜ける
		if (preKeys[DIK_ESCAPE] == 0 && keys[DIK_ESCAPE] != 0) {
			break;
		}
	}

	// ライブラリの終了
	Novice::Finalize();
	return 0;
}

#pragma region Functions

inline Vec3 Project(const Vec3& v1, const Vec3& v2) {
	Vec3 normalizedV2 = v2.Normalized();
	return normalizedV2 * (v1.Dot(normalizedV2));
}

float Clamp(const float value, const float min, const float max) {
	if (value < min) {
		return min;
	}
	if (value > max) {
		return max;
	}
	return value;
}

inline bool IsCollision(const Sphere& sphere1, const Sphere& sphere2) {
	return (sphere1.center - sphere2.center).Length() <= sphere1.radius + sphere2.radius;
}

inline bool IsCollision(const Sphere& sphere, const Plane& plane) {
	// 平面の法線ベクトルを正規化
	Vec3 normalizedNormal = plane.normal.Normalized();
	// 球の中心から平面までの距離を計算
	float distance = sphere.center.Dot(normalizedNormal) - plane.distance;
	// 距離が球の半径以下であれば衝突している
	return std::abs(distance) <= sphere.radius;
}

inline bool IsCollision(const Segment& segment, const Plane& plane) {
	// まず垂直判定を行うために、法線と線の内積を求める
	float dot = plane.normal.Dot(segment.diff);

	// 垂直=平行であるので、衝突しているはずがない
	if (dot == 0.0f) {
		return false;
	}

	// tを求める
	float t = (plane.distance - segment.origin.Dot(plane.normal)) / dot;

	return (t > 0.0f && t < 1.0f);
}

inline bool IsCollision(const Triangle& triangle, const Segment& segment) {
	Vec3 v0 = triangle.vertices[0];
	Vec3 v1 = triangle.vertices[1];
	Vec3 v2 = triangle.vertices[2];

	Vec3 v01 = v1 - v0;
	Vec3 v12 = v2 - v1;
	Vec3 v20 = v0 - v2;

	// 法線を求める
	Vec3 triNormal = v01.Cross(v12).Normalized();

	float dot = triNormal.Dot(segment.diff);

	if (dot == 0.0f) {
		return false;
	}

	Vec3 w0 = segment.origin - v0;

	// tを求める
	float t = -triNormal.Dot(w0) / dot;

	// tが0と1の間でなければ線分は三角形と交差しない
	if (t < 0.0f || t > 1.0f) {
		return false;
	}

	// 衝突点p!
	Vec3 p = segment.origin + segment.diff * t;

	// pが三角形の内側にあるか判定
	Vec3 c0 = v01.Cross(p - v0);
	Vec3 c1 = v12.Cross(p - v1);
	Vec3 c2 = v20.Cross(p - v2);

	if (triNormal.Dot(c0) >= 0.0f && triNormal.Dot(c1) >= 0.0f && triNormal.Dot(c2) >= 0.0f) {
		return true;
	}

	return false;
}

inline bool IsCollision(const AABB& a, const AABB& b) {
	return
		(a.min.x <= b.max.x && a.max.x >= b.min.x) &&
		(a.min.y <= b.max.y && a.max.y >= b.min.y) &&
		(a.min.z <= b.max.z && a.max.z >= b.min.z);
}

inline bool IsCollision(const AABB& aabb, const Sphere& sphere) {
	// 最近接点を求める
	Vec3 closestPoint = {
		std::clamp(sphere.center.x, aabb.min.x, aabb.max.x),
		std::clamp(sphere.center.y, aabb.min.y, aabb.max.y),
		std::clamp(sphere.center.z, aabb.min.z, aabb.max.z)
	};

	// 最近接点と球の中心との距離を求める
	float distance = closestPoint.Distance(sphere.center);
	// 距離が半径よりも小さければ衝突
	return distance <= sphere.radius;
}

inline bool IsCollision(const AABB& aabb, const Segment& segment) {
	Vec3 invDir = { 1.0f / segment.diff.x, 1.0f / segment.diff.y, 1.0f / segment.diff.z };

	Vec3 tMin = (aabb.min - segment.origin) * invDir;
	Vec3 tMax = (aabb.max - segment.origin) * invDir;

	float tNearX = min(tMin.x, tMax.x);
	float tNearY = min(tMin.y, tMax.y);
	float tNearZ = min(tMin.z, tMax.z);

	float tFarX = max(tMin.x, tMax.x);
	float tFarY = max(tMin.y, tMax.y);
	float tFarZ = max(tMin.z, tMax.z);

	float tmin = max(max(tNearX, tNearY), tNearZ);
	float tmax = min(min(tFarX, tFarY), tFarZ);

	if (tmin <= tmax && tmax >= 0.0f) {
		return true;
	}

	return false;
}

inline bool IsCollision(const OBB& obb, const Sphere& sphere) {
	// OBBのWorldMatrixを作る
	Mat4 obbWorld = {
		{
			{obb.orientations[0].x, obb.orientations[0].y, obb.orientations[0].z, 0.0f},
			{obb.orientations[1].x, obb.orientations[1].y, obb.orientations[1].z, 0.0f},
			{obb.orientations[2].x, obb.orientations[2].y, obb.orientations[2].z, 0.0f},
			{obb.center.x, obb.center.y, obb.center.z, 1.0f}
		}
	};

	Vec3 centerInOBBLocalSpace = Mat4::Transform(sphere.center, obbWorld.Inverse());

	AABB aabbOBBLocal = { {-obb.size.x, -obb.size.y, -obb.size.z}, obb.size };
	Sphere sphereOBBLocal = { centerInOBBLocalSpace, sphere.radius };

	return IsCollision(aabbOBBLocal, sphereOBBLocal);
}

inline bool IsCollision(const OBB& obb1, const OBB& obb2) {
	// それぞれのOBBの軸ベクトルを取得
	Vec3 axes1[] = { obb1.orientations[0], obb1.orientations[1], obb1.orientations[2] };
	Vec3 axes2[] = { obb2.orientations[0], obb2.orientations[1], obb2.orientations[2] };

	// OBB間のベクトルを計算
	Vec3 T = obb2.center - obb1.center;

	Vec3 axesToTest[15];
	int axisCount = 0;

	// OBB1の3つの軸
	for (auto i : axes1) {
		axesToTest[axisCount++] = i;
	}

	// OBB2の3つの軸
	for (auto i : axes2) {
		axesToTest[axisCount++] = i;
	}

	for (auto i : axes1) {
		for (int j = 0; j < 3; ++j) {
			axesToTest[axisCount++] = i.Cross(axes2[j]);
		}
	}

	// 各軸に対して分離軸を確認
	for (auto axis : axesToTest) {
		// 軸の長さがほぼゼロの場合（軸が有効でない場合）はスキップ
		if (axis.SqrLength() < std::numeric_limits<float>::epsilon()) {
			continue;
		}

		axis = axis.Normalized();

		// OBB1 の投影範囲
		float r1 = 0.0f;
		for (int j = 0; j < 3; ++j) {
			r1 += obb1.size[j] * fabs(axes1[j].Dot(axis));
		}

		// OBB2 の投影範囲
		float r2 = 0.0f;
		for (int j = 0; j < 3; ++j) {
			r2 += obb2.size[j] * fabs(axes2[j].Dot(axis));
		}

		// 軸に対する OBB 間の中心の距離
		float distance = fabs(T.Dot(axis));

		// 分離軸が存在する場合、衝突していない
		if (distance > r1 + r2) {
			return false;
		}
	}

	// 分離軸が存在しない場合、衝突している
	return true;
}


Vec3 Lerp(const Vec3& start, const Vec3& end, float t) {
	return start + (end - start) * t;
}

inline Vec3 Reflect(const Vec3& input, const Vec3& normal) {
	Vec3 normalizedNormal = normal.Normalized();
	return normalizedNormal * (input - 2.0f * input.Dot(normalizedNormal));
}

inline Vec3 ClosestPoint(const Vec3& point, const Segment& segment) {
	Vec3 segmentDir = point - segment.origin;
	float t = segmentDir.Dot(segment.diff) / segment.diff.SqrLength();
	t = Clamp(t, 0.0f, 1.0f);
	return segment.origin + segment.diff * t;
}

Vec3 Perpendicular(const Vec3& vector) {
	if (vector.x != 0.0f || vector.y != 0.0f) {
		return { -vector.y, vector.x, 0.0f };
	}
	return { 0.0f, -vector.z, vector.y };
}

inline void MatrixScreenPrintf(const int x, const int y, const Mat4& matrix, const char* label) {
	Novice::ScreenPrintf(x, y, "%s", label);
	for (int row = 0; row < 4; ++row) {
		for (int column = 0; column < 4; ++column) {
			Novice::ScreenPrintf(
				x + column * kColumnWidth, y + (row + 1) * kRowHeight, "%6.03f", matrix.m[row][column]
			);
		}
	}
}

inline void VectorScreenPrintf(const int x, const int y, const Vec3& vector, const char* label) {
	Novice::ScreenPrintf(x, y, "%6.02f %6.02f %6.02f %s", vector.x, vector.y, vector.z, label);
}

void QuaternionScreenPrintf(const int x, const int y, const Quaternion& quaternion, const char* label) {
	Novice::ScreenPrintf(x, y, "%6.02f %6.02f %6.02f %6.02f	: %s", quaternion.x, quaternion.y, quaternion.z, quaternion.w, label);
}

inline void DrawSphere(
	const Sphere& sphere, const Mat4& viewProjectionMatrix, const Mat4& viewportMatrix,
	uint32_t color
) {
	const uint32_t kSubdivision = 16; // 分割数
	const float kLonEvery = 360.0f / kSubdivision; // 経度分割1つ分の角度
	const float kLatEvery = 180.0f / kSubdivision; // 緯度分割1つ分の角度
	// 緯度の方向に分割
	for (uint32_t latIndex = 0; latIndex <= kSubdivision; ++latIndex) {
		float latAngle = -90.0f + kLatEvery * latIndex; // 現在の緯度
		float radLat = latAngle * Math::deg2Rad;
		// 経度の方向に分割 0 ~ 360度
		for (uint32_t lonIndex = 0; lonIndex <= kSubdivision; ++lonIndex) {
			float lonAngle = kLonEvery * lonIndex; // 現在の経度
			float radLon = lonAngle * Math::deg2Rad;
			// world座標系でのa,b,cを求める
			Vec3 a = {
				sphere.center.x + sphere.radius * cosf(radLat) * cosf(radLon),
				sphere.center.y + sphere.radius * sinf(radLat),
				sphere.center.z + sphere.radius * cosf(radLat) * sinf(radLon)
			};
			Vec3 b = {
				sphere.center.x + sphere.radius * cosf(radLat + kLatEvery * Math::deg2Rad) * cosf(radLon),
				sphere.center.y + sphere.radius * sinf(radLat + kLatEvery * Math::deg2Rad),
				sphere.center.z + sphere.radius * cosf(radLat + kLatEvery * Math::deg2Rad) * sinf(radLon)
			};
			Vec3 c = {
				sphere.center.x + sphere.radius * cosf(radLat) * cosf(radLon + kLonEvery * Math::deg2Rad),
				sphere.center.y + sphere.radius * sinf(radLat),
				sphere.center.z + sphere.radius * cosf(radLat) * sinf(radLon + kLonEvery * Math::deg2Rad)
			};

			// a,b,cをScreen座標系まで変換
			a = Mat4::Transform(a, viewProjectionMatrix * viewportMatrix);
			b = Mat4::Transform(b, viewProjectionMatrix * viewportMatrix);
			c = Mat4::Transform(c, viewProjectionMatrix * viewportMatrix);

			// ab, bcで線を引く
			Novice::DrawLine(
				static_cast<int>(a.x), static_cast<int>(a.y),
				static_cast<int>(b.x), static_cast<int>(b.y),
				color
			);

			Novice::DrawLine(
				static_cast<int>(a.x), static_cast<int>(a.y),
				static_cast<int>(c.x), static_cast<int>(c.y),
				color
			);
		}
	}
}

void DrawGrid(const Mat4& viewProjectionMatrix, const Mat4& viewportMatrix) {
	const float kGridHalfWidth = 2.0f; // Gridの半分の幅
	const uint32_t kSubdivision = 10; // 分割数
	const float kGridEvery = (kGridHalfWidth * 2.0f) / static_cast<float>(kSubdivision); // ひとつ分の長さ

	// 奥から手前への線を順々に引いていく
	for (uint32_t xIndex = 0; xIndex <= kSubdivision; ++xIndex) {
		// 上の情報を使ってワールド座標系上の視点と終点を求める
		float x = kGridHalfWidth - xIndex * kGridEvery;
		Vec3 start = { x, 0, -kGridHalfWidth };
		Vec3 end = { x, 0, kGridHalfWidth };
		// スクリーン座標まで変換をかける
		start = Mat4::Transform(start, viewProjectionMatrix * viewportMatrix);
		end = Mat4::Transform(end, viewProjectionMatrix * viewportMatrix);
		// 変換した座標を使って表示。色は薄い灰色(0xAAAAAAFF)、原点は黒ぐらいが良いが、何でも良い
		if (xIndex == kSubdivision / 2) {
			Novice::DrawLine(
				static_cast<int>(start.x), static_cast<int>(start.y),
				static_cast<int>(end.x), static_cast<int>(end.y),
				0x2222FFFF
			);
		} else {
			Novice::DrawLine(
				static_cast<int>(start.x), static_cast<int>(start.y),
				static_cast<int>(end.x), static_cast<int>(end.y),
				0xAAAAAAFF
			);
		}
	}

	// 左から右も同じように順々に引いていく
	for (uint32_t zIndex = 0; zIndex <= kSubdivision; ++zIndex) {
		float z = -kGridHalfWidth + zIndex * kGridEvery;
		Vec3 start = { -kGridHalfWidth, 0, z };
		Vec3 end = { kGridHalfWidth, 0, z };
		start = Mat4::Transform(start, viewProjectionMatrix * viewportMatrix);
		end = Mat4::Transform(end, viewProjectionMatrix * viewportMatrix);
		if (zIndex == kSubdivision / 2) {
			Novice::DrawLine(
				static_cast<int>(start.x), static_cast<int>(start.y),
				static_cast<int>(end.x), static_cast<int>(end.y),
				0xBB2222FF
			);
		} else {
			Novice::DrawLine(
				static_cast<int>(start.x), static_cast<int>(start.y),
				static_cast<int>(end.x), static_cast<int>(end.y),
				0xAAAAAAFF
			);
		}
	}
}

inline void DrawPlane(
	const Plane& plane, const Mat4& viewProjectionMatrix, const Mat4& viewportMatrix,
	const uint32_t color
) {
	Vec3 center = plane.normal * plane.distance; // 1
	Vec3 perpendiculars[4];
	perpendiculars[0] = Perpendicular(plane.normal).Normalized(); // 2
	perpendiculars[1] = { -perpendiculars[0].x, -perpendiculars[0].y, -perpendiculars[0].z }; // 3
	perpendiculars[2] = plane.normal.Cross(perpendiculars[0]); // 4
	perpendiculars[3] = { -perpendiculars[2].x, -perpendiculars[2].y, -perpendiculars[2].z }; // 5
	// 6
	Vec3 points[4];

	for (int32_t index = 0; index < 4; ++index) {
		Vec3 extend = perpendiculars[index] * 2.0f;
		Vec3 point = center + extend;
		points[index] = Mat4::Transform(Mat4::Transform(point, viewProjectionMatrix), viewportMatrix);
	}

	// pointsをそれぞれ結んでDrawLineで矩形を描画する。DrawTriangleを使って塗り潰ししても良いが、DepthがないのでMT3では分かりづらい
	Novice::DrawLine(
		static_cast<int>(points[0].x), static_cast<int>(points[0].y),
		static_cast<int>(points[2].x), static_cast<int>(points[2].y),
		color
	);

	Novice::DrawLine(
		static_cast<int>(points[1].x), static_cast<int>(points[1].y),
		static_cast<int>(points[2].x), static_cast<int>(points[2].y),
		color
	);

	Novice::DrawLine(
		static_cast<int>(points[1].x), static_cast<int>(points[1].y),
		static_cast<int>(points[3].x), static_cast<int>(points[3].y),
		color
	);

	Novice::DrawLine(
		static_cast<int>(points[3].x), static_cast<int>(points[3].y),
		static_cast<int>(points[0].x), static_cast<int>(points[0].y),
		color
	);
}

inline void DrawTriangle(
	const Triangle& triangle, const Mat4& viewProjectionMatrix, const Mat4& viewportMatrix,
	uint32_t color
) {
	Vec3 points[3];

	for (int i = 0; i < 3; ++i) {
		points[i] = Mat4::Transform(triangle.vertices[i], viewProjectionMatrix * viewportMatrix);
	}

	Novice::DrawLine(
		static_cast<int>(points[0].x), static_cast<int>(points[0].y),
		static_cast<int>(points[1].x), static_cast<int>(points[1].y),
		color
	);

	Novice::DrawLine(
		static_cast<int>(points[1].x), static_cast<int>(points[1].y),
		static_cast<int>(points[2].x), static_cast<int>(points[2].y),
		color
	);

	Novice::DrawLine(
		static_cast<int>(points[2].x), static_cast<int>(points[2].y),
		static_cast<int>(points[0].x), static_cast<int>(points[0].y),
		color
	);
}

inline void DrawAABB(const AABB& aabb, const Mat4& viewProjectionMatrix, const Mat4& viewportMatrix, uint32_t color) {
	Vec3 points[8];

	points[0] = Mat4::Transform({ aabb.min.x, aabb.min.y, aabb.min.z }, viewProjectionMatrix * viewportMatrix);
	points[1] = Mat4::Transform({ aabb.max.x, aabb.min.y, aabb.min.z }, viewProjectionMatrix * viewportMatrix);
	points[2] = Mat4::Transform({ aabb.max.x, aabb.max.y, aabb.min.z }, viewProjectionMatrix * viewportMatrix);
	points[3] = Mat4::Transform({ aabb.min.x, aabb.max.y, aabb.min.z }, viewProjectionMatrix * viewportMatrix);
	points[4] = Mat4::Transform({ aabb.min.x, aabb.min.y, aabb.max.z }, viewProjectionMatrix * viewportMatrix);
	points[5] = Mat4::Transform({ aabb.max.x, aabb.min.y, aabb.max.z }, viewProjectionMatrix * viewportMatrix);
	points[6] = Mat4::Transform({ aabb.max.x, aabb.max.y, aabb.max.z }, viewProjectionMatrix * viewportMatrix);
	points[7] = Mat4::Transform({ aabb.min.x, aabb.max.y, aabb.max.z }, viewProjectionMatrix * viewportMatrix);

	Novice::DrawLine(
		static_cast<int>(points[0].x), static_cast<int>(points[0].y), static_cast<int>(points[1].x),
		static_cast<int>(points[1].y), color
	);
	Novice::DrawLine(
		static_cast<int>(points[1].x), static_cast<int>(points[1].y), static_cast<int>(points[2].x),
		static_cast<int>(points[2].y), color
	);
	Novice::DrawLine(
		static_cast<int>(points[2].x), static_cast<int>(points[2].y), static_cast<int>(points[3].x),
		static_cast<int>(points[3].y), color
	);
	Novice::DrawLine(
		static_cast<int>(points[3].x), static_cast<int>(points[3].y), static_cast<int>(points[0].x),
		static_cast<int>(points[0].y), color
	);

	Novice::DrawLine(
		static_cast<int>(points[4].x), static_cast<int>(points[4].y), static_cast<int>(points[5].x),
		static_cast<int>(points[5].y), color
	);
	Novice::DrawLine(
		static_cast<int>(points[5].x), static_cast<int>(points[5].y), static_cast<int>(points[6].x),
		static_cast<int>(points[6].y), color
	);
	Novice::DrawLine(
		static_cast<int>(points[6].x), static_cast<int>(points[6].y), static_cast<int>(points[7].x),
		static_cast<int>(points[7].y), color
	);
	Novice::DrawLine(
		static_cast<int>(points[7].x), static_cast<int>(points[7].y), static_cast<int>(points[4].x),
		static_cast<int>(points[4].y), color
	);

	Novice::DrawLine(
		static_cast<int>(points[0].x), static_cast<int>(points[0].y), static_cast<int>(points[4].x),
		static_cast<int>(points[4].y), color
	);
	Novice::DrawLine(
		static_cast<int>(points[1].x), static_cast<int>(points[1].y), static_cast<int>(points[5].x),
		static_cast<int>(points[5].y), color
	);
	Novice::DrawLine(
		static_cast<int>(points[2].x), static_cast<int>(points[2].y), static_cast<int>(points[6].x),
		static_cast<int>(points[6].y), color
	);
	Novice::DrawLine(
		static_cast<int>(points[3].x), static_cast<int>(points[3].y), static_cast<int>(points[7].x),
		static_cast<int>(points[7].y), color
	);
}

inline void DrawSegment(
	const Segment& segment, const Mat4& viewProjectionMatrix, const Mat4& viewportMatrix, const unsigned int color
) {
	Vec3 org = Mat4::Transform(segment.origin, viewProjectionMatrix * viewportMatrix);
	Vec3 diff = Mat4::Transform(segment.origin + segment.diff, viewProjectionMatrix * viewportMatrix);

	Novice::DrawLine(
		static_cast<int>(org.x), static_cast<int>(org.y),
		static_cast<int>(diff.x), static_cast<int>(diff.y),
		color
	);
}

void DrawLine(
	const Vec3& start, const Vec3& end, const Mat4& viewProjectionMatrix,
	const Mat4& viewportMatrix, unsigned int color
) {
	Vec3 pStart = Mat4::Transform(start, viewProjectionMatrix * viewportMatrix);
	Vec3 pEnd = Mat4::Transform(end, viewProjectionMatrix * viewportMatrix);

	Novice::DrawLine(
		static_cast<int>(pStart.x), static_cast<int>(pStart.y),
		static_cast<int>(pEnd.x), static_cast<int>(pEnd.y),
		color
	);
}

inline void DrawObb(const OBB& obb, const Mat4& viewProjectionMatrix, const Mat4& viewportMatrix, uint32_t color) {
	// OBBのWorldMatrixを作成
	Mat4 obbWorld = {
		{
			{obb.orientations[0].x, obb.orientations[0].y, obb.orientations[0].z, 0.0f},
			{obb.orientations[1].x, obb.orientations[1].y, obb.orientations[1].z, 0.0f},
			{obb.orientations[2].x, obb.orientations[2].y, obb.orientations[2].z, 0.0f},
			{obb.center.x, obb.center.y, obb.center.z, 1.0f}
		}
	};

	// 8つの頂点を計算
	Vec3 vertices[8] = {
		{-obb.size.x, -obb.size.y, -obb.size.z},
		{obb.size.x, -obb.size.y, -obb.size.z},
		{obb.size.x, obb.size.y, -obb.size.z},
		{-obb.size.x, obb.size.y, -obb.size.z},
		{-obb.size.x, -obb.size.y, obb.size.z},
		{obb.size.x, -obb.size.y, obb.size.z},
		{obb.size.x, obb.size.y, obb.size.z},
		{-obb.size.x, obb.size.y, obb.size.z}
	};

	// 頂点をワールド座標系に変換
	for (Vec3& vertex : vertices) {
		vertex = Mat4::Transform(vertex, obbWorld);
		vertex = Mat4::Transform(vertex, viewProjectionMatrix * viewportMatrix);
	}

	// OBBのエッジを描画
	static const int EDGES[12][2] = {
		{0, 1}, {1, 2}, {2, 3}, {3, 0}, // 底面
		{4, 5}, {5, 6}, {6, 7}, {7, 4}, // 上面
		{0, 4}, {1, 5}, {2, 6}, {3, 7} // 側面
	};

	for (const int* const edge : EDGES) {
		Novice::DrawLine(
			static_cast<int>(vertices[edge[0]].x), static_cast<int>(vertices[edge[0]].y),
			static_cast<int>(vertices[edge[1]].x), static_cast<int>(vertices[edge[1]].y),
			color
		);
	}
}

inline void DrawBezier(
	const Vec3& controlPoint0,
	const Vec3& controlPoint1,
	const Vec3& controlPoint2,
	const Mat4& viewProjectionMatrix,
	const Mat4& viewportMatrix,
	uint32_t color
) {
	constexpr float res = 0.025f;
	for (float t = 0.0f; t <= 1.0f; t += res) {
		float nextT = t + res;
		if (nextT > 1.0f) {
			nextT = 1.0f;
		}

		// 2次ベジエ曲線の幾何学的解釈を使用
		Vec3 p0p1 = Lerp(controlPoint0, controlPoint1, t);
		Vec3 p1p2 = Lerp(controlPoint1, controlPoint2, t);
		Vec3 p0 = Lerp(p0p1, p1p2, t);

		// 次の点を計算
		Vec3 p0p1Next = Lerp(controlPoint0, controlPoint1, nextT);
		Vec3 p1p2Next = Lerp(controlPoint1, controlPoint2, nextT);
		Vec3 p1 = Lerp(p0p1Next, p1p2Next, nextT);

		Vec3 point0 = Mat4::Transform(p0, viewProjectionMatrix * viewportMatrix);
		Vec3 point1 = Mat4::Transform(p1, viewProjectionMatrix * viewportMatrix);

		// 線を描画
		Novice::DrawLine(
			static_cast<int>(point0.x), static_cast<int>(point0.y),
			static_cast<int>(point1.x), static_cast<int>(point1.y),
			color
		);
	}
}

inline void DrawCatmullRom(
	const Vec3& controlPoint0,
	const Vec3& controlPoint1,
	const Vec3& controlPoint2,
	const Vec3& controlPoint3,
	const Mat4& viewProjectionMatrix,
	const Mat4& viewportMatrix,
	uint32_t color
) {
	constexpr float res = 0.025f;
	for (float t = 0.0f; t <= 1.0f; t += res) {
		float nextT = t + res;
		if (nextT > 1.0f) {
			nextT = 1.0f;
		}

		// Catmull-Romスプラインの計算
		Vec3 p0 = Cubic(controlPoint0, controlPoint1, controlPoint2, controlPoint3, t);
		Vec3 p1 = Cubic(controlPoint0, controlPoint1, controlPoint2, controlPoint3, nextT);

		Vec3 point0 = Mat4::Transform(p0, viewProjectionMatrix * viewportMatrix);
		Vec3 point1 = Mat4::Transform(p1, viewProjectionMatrix * viewportMatrix);

		// 線を描画
		Novice::DrawLine(
			static_cast<int>(point0.x), static_cast<int>(point0.y),
			static_cast<int>(point1.x), static_cast<int>(point1.y),
			color
		);
	}
}
#pragma endregion
