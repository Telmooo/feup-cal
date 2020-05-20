#ifndef MEATWAGONS_REQUEST_H
#define MEATWAGONS_REQUEST_H

class Request {
    int numPris;
    int type;
    int pickup;
    int dest;
    double p_dist;
    double p_time;

    int pickupHour;
    int destHour;
public:
    int getPickupHour() const;

    void setPickupHour(int pickupHour);

    int getDestHour() const;

    void setDestHour(int destHour);

public:
    Request(int numPris, int type, int pickup, int dest, double pDist, double pTime);

    int getNumPris() const;

    void setNumPris(int numPris);

    int getType() const;

    void setType(int type);

    int getPickup() const;

    void setPickup(int pickup);

    int getDest() const;

    void setDest(int dest);

    double getPDist() const;

    void setPDist(double pDist);

    double getPTime() const;

    void setPTime(double pTime);
};

#endif //MEATWAGONS_REQUEST_H
