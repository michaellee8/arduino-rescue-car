package org.tensorflow.lite.examples.poseestimation.data

enum class CarDirection(val num: Int) {
    LEFT(0),
    FORWARD(1),
    RIGHT(2),
    BACKWARD(4),
    STOP(5)
}
