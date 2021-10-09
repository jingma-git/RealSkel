#pragma once

#include <QVector3D>
#include <QQuaternion>
#include <QTime>

#include <cmath>

class TrackBall
{
public:
    TrackBall();

    void mouseMove(const QVector3D &p);
    void mousePress(const QVector3D &p);
    void mouseRelease(const QVector3D &p);
    void resizeViewport(int width, int height);
    QQuaternion getRotation();

private:
    QQuaternion rotation;
    QVector3D axis;
    double velocity;

    QVector3D lastPos3D;
    QTime lastTime;
    bool trackingMouse;

    double viewportWidth;
    double viewportHeight;

    const double rad2deg;
};
