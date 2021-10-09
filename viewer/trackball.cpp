#include "trackball.h"

TrackBall::TrackBall() : rad2deg(180.0 / M_PI)
{
    velocity = 0.0;
    trackingMouse = false;
    lastTime = QTime ::currentTime();
}

void TrackBall ::resizeViewport(int width, int height)
{
    viewportWidth = static_cast<double>(width);
    viewportHeight = static_cast<double>(height);
}

QQuaternion TrackBall ::getRotation()
{
    if (trackingMouse)
        return rotation;

    QTime currentTime = QTime ::currentTime();
    double angle = velocity * lastTime.msecsTo(currentTime);
    return QQuaternion ::fromAxisAndAngle(axis, angle) * rotation;
}

void TrackBall::mouseMove(const QVector3D &p)
{
    if (!trackingMouse)
        return;

    QTime currentTime = QTime ::currentTime();
    int msecs = lastTime.msecsTo(currentTime);
    if (msecs)
    {
        QVector3D vp = p;
        QVector3D currentPos3D = QVector3D(vp.x(), vp.y(), 0.0);
        double lenSqr = currentPos3D.lengthSquared();

        if (lenSqr >= 1.0)
            currentPos3D.normalize();
        else
            currentPos3D.setZ(sqrt(1.0 - lenSqr));

        axis = QVector3D::crossProduct(lastPos3D, currentPos3D);

        double angle = rad2deg * axis.length();
        velocity = angle / msecs;
        axis.normalize();
        rotation = QQuaternion ::fromAxisAndAngle(axis, angle) * rotation;

        lastPos3D = currentPos3D;
        lastTime = currentTime;
    }
}

void TrackBall::mousePress(const QVector3D &p)
{
    rotation = getRotation();
    trackingMouse = true;
    lastTime = QTime ::currentTime();

    lastPos3D = p;
    double lenSqr = lastPos3D.lengthSquared();

    if (lenSqr >= 1.0)
        lastPos3D.normalize();
    else
        lastPos3D.setZ(sqrt(1.0 - lenSqr));

    velocity = 0.0;
}

void TrackBall::mouseRelease(const QVector3D &p)
{
    mouseMove(p);
    trackingMouse = false;
}
