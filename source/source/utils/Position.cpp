#include "Position.h"

#include <cmath>

Position::Position() : x(0.0), y(0.0) { }

Position::Position(double x, double y) : x(x), y(y) { }

Position::Position(const Position &position) {
    this->x = position.x;
    this->y = position.y;
}

double Position::getX() const { return x; }

double Position::getY() const { return y; }

void Position::setX(double x) { this->x = x; }

void Position::setY(double y) { this->y = y; }

double Position::distance(const Position &position) const {
    return sqrt(pow(this->x - position.x, 2) + pow(this->y - position.y, 2));
}
