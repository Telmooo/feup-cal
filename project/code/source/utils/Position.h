#ifndef MEATWAGONS_POSITION_H
#define MEATWAGONS_POSITION_H

class Position {
    double x;                      // x position
    double y;                      // y position
public:
    Position();

    Position(double x, double y);

    Position(const Position &position);

    double getX() const;

    double getY() const;

    void setX(double x);

    void setY(double y);

    double distance(const Position &position) const;
};

#endif //MEATWAGONS_POSITION_H
