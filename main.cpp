#include "pr.hpp"
#include <iostream>
#include <memory>

template <class T>
inline T ss_max(T x, T y)
{
    return (x < y) ? y : x;
}

template <class T>
inline T ss_min(T x, T y)
{
    return (y < x) ? y : x;
}

float sign_of(float v)
{
    return v < 0.0f ? -1.0f : 1.0f;
}

// ax^2 + bx + c == 0
int solve_quadratic(float xs[2], float a, float b, float c)
{
    float det = b * b - 4.0f * a * c;
    if (det < 0.0f)
    {
        return 0;
    }

    float k = (-b - sign_of(b) * std::sqrtf(det)) / 2.0f;
    float x0 = k / a;
    float x1 = c / k;
    xs[0] = ss_min(x0, x1);
    xs[1] = ss_max(x0, x1);
    return 2;
}

float intersect_ray_ellipsoid( glm::vec3 U, glm::vec3 V, glm::vec3 W, glm::vec3 ro, glm::vec3 rd )
{
    glm::vec3 u = U / glm::dot(U, U);
    glm::vec3 v = V / glm::dot(V, V);
    glm::vec3 w = W / glm::dot(W, W);

    auto sqr = [](float x) { return x * x; };

    float k = 1.0f;
    float t_delta = -glm::dot(rd, ro) / glm::dot(rd, rd);
    glm::vec3 ro_prime = ro + rd * t_delta;

    float urd = glm::dot(u, rd);
    float vrd = glm::dot(v, rd);
    float wrd = glm::dot(w, rd);
    float uro = glm::dot(u, ro_prime);
    float vro = glm::dot(v, ro_prime);
    float wro = glm::dot(w, ro_prime);
    float A = sqr(urd) + sqr(vrd) + sqr(wrd);
    float B = 2.0f * (urd * uro + vrd * vro + wrd * wro);
    float C = sqr(uro) + sqr(vro) + sqr(wro) - k * k;

    float xs[2];
    if (solve_quadratic(xs, A, B, C))
    {
        return xs[0] + t_delta;
    }
    return -1.0f;
}

int main() {
    using namespace pr;

    SetDataDir(ExecutableDir());

    Config config;
    config.ScreenWidth = 1920;
    config.ScreenHeight = 1080;
    config.SwapInterval = 1;
    Initialize(config);

    Camera3D camera;
    camera.origin = { 4, 4, 4 };
    camera.lookat = { 0, 0, 0 };
    camera.zNear = 1.0f;
    camera.zFar = 1000.0f;

    //camera.origin *= 100.0f;
    //camera.fovy = 0.005f;

    double e = GetElapsedTime();

    ITexture *tex = CreateTexture();
    Image2DRGBA8 image;
    int stride = 1;

    Image2DRGBA8 earth;
    earth.load("earth.jpg");

    while (pr::NextFrame() == false) {
        if (IsImGuiUsingMouse() == false) {
            UpdateCameraBlenderLike(&camera);
        }

        // ClearBackground(0.1f, 0.1f, 0.1f, 1);
        ClearBackground(tex);

        BeginCamera(camera);

        PushGraphicState();

        DrawGrid(GridAxis::XZ, 1.0f, 10, { 128, 128, 128 });

        static glm::vec3 U = { 1, 0, 0 };
        static glm::vec3 V = { 0, 1, 0 };
        static glm::vec3 W = { 0, 0, 1 };
        ManipulatePosition(camera, &U, 0.2f);
        ManipulatePosition(camera, &V, 0.2f);
        ManipulatePosition(camera, &W, 0.2f);

        {
            glm::vec3 N = glm::cross(U, V);
            V = glm::normalize(glm::cross(N, U)) * glm::length(V);

            DrawArrow({ 0,0,0 }, U, 0.01f, { 255,0,0 });
            DrawArrow({ 0,0,0 }, V, 0.01f, { 0,255,0 });
            DrawArrow({ 0,0,0 }, W, 0.01f, { 0,0,255 });

            W = glm::normalize(N) * glm::length(W);
        }

        
        image.allocate(GetScreenWidth() / stride, GetScreenHeight() / stride);
        CameraRayGenerator rayGenerator(GetCurrentViewMatrix(), GetCurrentProjMatrix(), image.width(), image.height());

        for (int j = 0; j < image.height(); ++j)
        {
            for (int i = 0; i < image.width(); ++i)
            {
                glm::vec3 ro, rd;
                rayGenerator.shoot(&ro, &rd, i, j, 0.5f, 0.5f);

                float t = intersect_ray_ellipsoid(U, V, W, ro, rd);
                if (0.0f < t)
                {
                    glm::vec3 p = ro + rd * t;
                    float X = glm::dot(U, p) / glm::dot(U, U);
                    float Y = glm::dot(V, p) / glm::dot(V, V);
                    float Z = glm::dot(W, p) / glm::dot(W, W);

                    float u = glm::fract( std::atan2(-Z, X) / (2.0f * glm::pi<float>()) );
                    float v = std::atan2(std::sqrt(X * X + Z * Z), Y) / (glm::pi<float>());

                    int xi = glm::clamp( (int)( earth.width() * u ), 0, earth.width() - 1);
                    int yi = glm::clamp( (int)( earth.height() * v ), 0, earth.width() - 1);
                    image(i, j) = earth(xi, yi);
                }
                else
                {
                    image(i, j) = { 0, 0, 0, 255 };
                }
            }
        }
        tex->upload(image);

        // https://tavianator.com/2014/ellipsoid_bounding_boxes.html
        float dx = std::sqrt( U.x * U.x + V.x * V.x + W.x * W.x );
        float dy = std::sqrt( U.y * U.y + V.y * V.y + W.y * W.y );
        float dz = std::sqrt( U.z * U.z + V.z * V.z + W.z * W.z );
        DrawCube({ 0, 0, 0 }, { dx * 2, dy * 2, dz * 2 }, { 255,255,255 });

        PopGraphicState();
        EndCamera();

        BeginImGui();

        ImGui::SetNextWindowSize({ 500, 800 }, ImGuiCond_Once);
        ImGui::Begin("Panel");
        ImGui::Text("fps = %f", GetFrameRate());

        ImGui::SliderFloat("fov", &camera.fovy, 0, 0.1);
        ImGui::Text("cam d %f", glm::length(camera.origin));

        ImGui::End();

        EndImGui();
    }

    pr::CleanUp();
}
