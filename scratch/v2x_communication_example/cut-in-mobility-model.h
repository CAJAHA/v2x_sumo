#ifndef CUT_IN_MOBILITY_MODEL_H
#define CUT_IN_MOBILITY_MODEL_H

#include "ns3/mobility-model.h"
#include "ns3/nstime.h"
#include "ns3/random-variable-stream.h"
#include "ns3/output-stream-wrapper.h"
#include <list>
#include <map>
#include <stdlib.h>

namespace ns3 {


class CutInMobilityModel : public MobilityModel 
{
public:
    struct VehPosition
    {
        double m_x;
        double m_y;
    };

    typedef VehPosition VehVelocity; 

    struct VehState
    {
        int64_t m_genTime;
        uint32_t m_ID;
        VehPosition m_position;
        VehVelocity m_velocity;
    };

    struct Action
    {
        Vector m_acc;
        double m_steer;
    };

    enum TrafficLevel
    {
        L1, // (3 ~ 4 vehicles/km, average velocity 25.56)
        L2, // (4 ~ 6 vehicles/km, average velocity 22.56)
        L3  // (6 ~ 7 vehicles/km, average velocity 18.89)
    };

    enum BelongTo
    {
        MAIN_ROAD,
        MERGE_ROAD
    };

    static TypeId GetTypeId (void);
    static std::list<VehPosition> GetInitPosition(TrafficLevel level, BelongTo road);
    static void PrintPositionError(ns3::Ptr<ns3::OutputStreamWrapper> log_stream);

    CutInMobilityModel ();
    virtual ~CutInMobilityModel ();

    void DoPassVehState(const VehState&);
    VehState DoGetVehState();

    void ScheduleToStart(uint32_t ID, uint32_t start_ms, TrafficLevel level, BelongTo road);

    static const uint32_t BUFFER_LENGTH = 20;
    static double SQRT3;
    static double SQRT3_2;

private:
    void StartToMove(double start_velocity);
    void ChangeAction();
    Action GetAction();
    void UpdateVehStates();

    void CalculatePositionError();

    virtual Vector DoGetPosition (void) const;
    virtual void DoSetPosition (const Vector &position);
    virtual Vector DoGetVelocity (void) const;

    VehState m_vehstates[BUFFER_LENGTH];

    Time m_baseTime;  //!< the base time
    Vector m_basePosition; //!< the base position
    Vector m_baseVelocity; //!< the base velocity
    Vector m_acceleration;  //!< the acceleration

    uint32_t m_ID;
    BelongTo m_road;

    
    static std::list<CutInMobilityModel*> CutInMobilityModels;
    static int PositionError[3][5];
    static int PositionErrorCount[3];
    static double PositionErrorValue[3][5];
};

} // namespace ns3

#endif 