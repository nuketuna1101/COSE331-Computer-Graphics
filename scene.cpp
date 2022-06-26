#include "scene.h"

#include "obj_teapot.h"
#include "tex_flower.h"
#include <cmath>

Shader* Scene::vertexShader = nullptr;
Shader* Scene::fragmentShader = nullptr;
Program* Scene::program = nullptr;
Camera* Scene::camera = nullptr;
Light* Scene::light = nullptr;
Object* Scene::teapot = nullptr;
Material* Scene::flower = nullptr;

void Scene::setup(AAssetManager* aAssetManager) {

    // set asset manager
    Asset::setManager(aAssetManager);

    // create shaders
    vertexShader = new Shader(GL_VERTEX_SHADER, "vertex.glsl");
    fragmentShader = new Shader(GL_FRAGMENT_SHADER, "fragment.glsl");

    // create program
    program = new Program(vertexShader, fragmentShader);

    // create camera
    camera = new Camera(program);
    camera->eye = vec3(60.0f, 00.0f, 0.0f);
    camera->updateCameraUVN();

    // create light
    light = new Light(program);
    light->position = vec3(100.0f, 0.0f, 0.0f);

    // create floral texture
    flower = new Material(program, texFlowerData, texFlowerSize);

    // create teapot object
    teapot = new Object(program, flower, objTeapotVertices, objTeapotIndices,
                        objTeapotVerticesSize, objTeapotIndicesSize);

    //////////////////////////////
    /* TODO: Problem 2.
     *  Scale the teapot by 2.0 along the y-axis.
     *  Rotate the teapot by 90° CW about the rotation axis defined by two points
     *  (0, 0, 10) → (10, 0, 20).
     */
    mat4 scaleM, rotMat;
    mat4 T1, R1, RzCW90, R2, T2;
    float iofsqrt2 = 1.0 / sqrt(2);
    scaleM = transpose(mat4(1.0f, 0.0f, 0.0f, 0.0f,  // In OpenGL, the matrix must be transposed
                            0.0f, 2.0f, 0.0f, 0.0f,
                            0.0f, 0.0f, 1.0f, 0.0f,
                            0.0f, 0.0f, 0.0f, 1.0f));
    // 5 matrices combination
    T1 = transpose(mat4(1.0f, 0.0f, 0.0f, -10.0f,
                        0.0f, 1.0f, 0.0f, 0.0f,
                        0.0f, 0.0f, 1.0f, 0.0f,
                        0.0f, 0.0f, 0.0f, 1.0f));
    R1 = transpose(mat4(0.0f, iofsqrt2 * (-1.0f), iofsqrt2 * 1.0f, 0.0f,
                        1.0f, 0.0f, 0.0f, 0.0f,
                        0.0f, iofsqrt2 * 1.0f, iofsqrt2 * 1.0f, 0.0f,
                        0.0f, 0.0f, 0.0f, 1.0f));
    RzCW90 = transpose(mat4(0.0f, 1.0f, 0.0f, 0.0f,
                        -1.0f, 0.0f, 0.0f, 0.0f,
                        0.0f, 0.0f, 1.0f, 0.0f,
                        0.0f, 0.0f, 0.0f, 1.0f));
    R2 = transpose(mat4(0.0f, 1.0f, 0.0f, 0.0f,
                        iofsqrt2 * (-1.0f), 0.0f, iofsqrt2 * 1.0f, 0.0f,
                        iofsqrt2 * 1.0f, 0.0f, iofsqrt2 * 1.0f, 0.0f,
                        0.0f, 0.0f, 0.0f, 1.0f));
    T2 = transpose(mat4(1.0f, 0.0f, 0.0f, 10.0f,
                        0.0f, 1.0f, 0.0f, 0.0f,
                        0.0f, 0.0f, 1.0f, 0.0f,
                        0.0f, 0.0f, 0.0f, 1.0f));
    rotMat = T1 * R1 * RzCW90 * R2 * T2;
    teapot->worldMatrix = rotMat * scaleM * teapot->worldMatrix;
    //////////////////////////////
}

void Scene::screen(int width, int height) {

    // set camera aspect ratio
    camera->aspect = (float) width / height;
}

void Scene::update(float deltaTime) {
    // use program
    program->use();

    //////////////////////////////
    /* TODO: Problem 3.
     *  Rotate the teapot about the z-axis.
     */
    float sinx = sin(deltaTime);
    float cosx = cos(deltaTime);
    mat4 RzCCW = transpose(mat4(cosx * 1.0f, sinx * (-1.0f), 0.0f, 0.0f,
                                sinx * 1.0f, cosx * 1.0f, 0.0f, 0.0f,
                                0.0f, 0.0f, 1.0f, 0.0f,
                                0.0f, 0.0f, 0.0f, 1.0f));
    teapot->worldMatrix = RzCCW * teapot->worldMatrix;
    //////////////////////////////


    camera->updateViewMatrix();
    camera->updateProjectionMatrix();
    light->setup();


    // draw teapot
    teapot->draw();
}

void Scene::rotateCamera(float dx,float dy) {
    float rotationSensitivity = 0.03;

    float thetaYaw=glm::radians(rotationSensitivity*dx);
    float thetaPinch=glm::radians(rotationSensitivity*dy);

    rotateCameraYaw(thetaYaw);
    rotateCameraPitch(thetaPinch);
}

void Scene::rotateCameraYaw(float theta) {

    //////////////////////////////
    /* TODO: Problem 4.
     *  calculate the rotated u,n vector abou   t v axis.
     *  Argument theta is amount of rotation in radians. theta is positive when CCW.
     *  Note that u,v,n should always be orthonormal.
     *  The u vector can be accessed via camera->cameraU.
     *  The v vector can be accessed via camera->cameraV.
     *  The n vector can be accessed via camera->cameraN.
     */
    mat4 RvTheta = transpose(mat4(cos(theta) * 1.0f, 0.0f, sin(theta) * 1.0f, 0.0f,
                                  0.0f, 1.0f, 0.0f, 0.0f,
                                  sin(theta) * (-1.0f), 0.0f, cos(theta) * 1.0f, 0.0f,
                                  0.0f, 0.0f, 0.0f, 1.0f));
    //mat4 rotate_yaw = rotate(mat4(1.0f), theta, vec3(0.0f, 1.0f, 0.0f));
    camera->cameraU = RvTheta * vec4(camera->cameraU, 1.0);
    camera->cameraN = RvTheta * vec4(camera->cameraN, 1.0);
    camera->updateViewMatrix();
    //////////////////////////////
}

void Scene::rotateCameraPitch(float theta) {

    //////////////////////////////
    /* TODO: Problem 4.
     *  calculate the rotated v,n vector about u axis.
     *  Argument theta is amount of rotation in radians. Theta is positive when CCW.
     *  Note that u,v,n should always be orthonormal.
     *  The u vector can be accessed via camera->cameraU.
     *  The v vector can be accessed via camera->cameraV.
     *  The n vector can be accessed via camera->cameraN.
     */
    mat4 RuTheta = transpose(mat4(cos(theta) * 1.0f, sin(theta) * (1.0f), 0.0f, 0.0f,
                                  sin(theta) * (-1.0f), cos(theta) * 1.0f, 0.0f, 0.0f,
                                  0.0f, 0.0f, 1.0f, 0.0f,
                                  0.0f, 0.0f, 0.0f, 1.0f));
    //mat4 rotate_pitch = rotate(mat4(1.0f), -theta, vec3(0.0f, 0.0f, 1.0f));
    camera->cameraV = RuTheta * vec4(camera->cameraV, 1.0);
    camera->cameraN = RuTheta * vec4(camera->cameraN, 1.0);
    camera->updateViewMatrix();
    //////////////////////////////
}


void Scene::translateLeft(float amount) {

    //////////////////////////////
    /* TODO: Problem 4.
     *  Calculate the camera position(eye) when translated left.
     */
    mat4 Tleft = translate(mat4(), vec3(0.0f, 0.0f, 1.0f));
    camera->eye = Tleft * vec4(camera->eye, 1.0);
    camera->updateViewMatrix();
    //////////////////////////////
}

void Scene::translateFront(float amount) {

    //////////////////////////////
    /* TODO: Problem 4.
     *  Calculate the camera position(eye) when translated front.
     */
    mat4 Tzin = translate(mat4(), vec3(-1.0f, 0.0f, 0.0f));
    camera->eye = Tzin * vec4(camera->eye, 1.0);
    camera->updateViewMatrix();
    //////////////////////////////
}

void Scene::translateRight(float amount) {

    //////////////////////////////
    /* TODO: Problem 4.
     *  Calculate the camera position(eye) when translated right.
     */
   mat4 Tright = translate(mat4(), vec3(0.0f, 0.0f, -1.0f));
    camera->eye = Tright * vec4(camera->eye, 1.0);
    camera->updateViewMatrix();
    //////////////////////////////
}

void Scene::translateBack(float amount) {

    //////////////////////////////
    /* TODO: Problem 4.
     *  Calculate the camera position(eye) when translated back.
     */
    mat4 Tzout = translate(mat4(), vec3(1.0f, 0.0f, 0.0f));
    camera->eye = Tzout * vec4(camera->eye, 1.0);
    camera->updateViewMatrix();
    //////////////////////////////
}
