#include "cut-in-mobility-model.h"
#include "ns3/simulator.h"
#include <list>

namespace ns3 {

NS_OBJECT_ENSURE_REGISTERED (CutInMobilityModel);


std::list<CutInMobilityModel*> CutInMobilityModel::CutInMobilityModels;
double CutInMobilityModel::PositionError[3];
int CutInMobilityModel::PositionErrorCount[3];

double CutInMobilityModel::SQRT3 = 1.7320508075688772;
double CutInMobilityModel::SQRT3_2 = 1.7320508075688772 / 2;


TypeId CutInMobilityModel::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::CutInMobilityModel")
    .SetParent<MobilityModel> ()
    .SetGroupName ("Mobility")
    .AddConstructor<CutInMobilityModel> ();
  return tid;
}

std::list<CutInMobilityModel::VehPosition> 
CutInMobilityModel::GetInitPosition (CutInMobilityModel::TrafficLevel level, CutInMobilityModel::BelongTo road)
{
    std::list<CutInMobilityModel::VehPosition> return_list;
    Ptr<ns3::UniformRandomVariable> RV = CreateObject<UniformRandomVariable> ();
    // VehPosition pos;
    int vehnum;
    double interval;
    if (road == BelongTo::MAIN_ROAD)
    {
        switch (level)
        {
        case TrafficLevel::L1:
            vehnum = rand() % (4-3+1) + 3;
            interval = 200.0 / double(vehnum - 1);
            for (int i = 0; i < vehnum; i++)
            {
                VehPosition pos = {-575.0 + i*interval + RV->GetValue(-0.1*interval, 0.1*interval), 0.0};
                return_list.push_back(pos);
            }
            break;
        case TrafficLevel::L2:
            vehnum = rand() % (6-4+1) + 4;
            interval = 200.0 / double(vehnum - 1);
            for (int i = 0; i < vehnum; i++)
            {
                VehPosition pos = {-575.0 + i*interval + RV->GetValue(-0.1*interval, 0.1*interval), 0.0};
                return_list.push_back(pos);
            }
            break;
        case TrafficLevel::L3:
            vehnum = rand() % (7-6+1) + 6;
            interval = 200.0 / double(vehnum - 1);
            for (int i = 0; i < vehnum; i++)
            {
                VehPosition pos = {-575.0 + i*interval + RV->GetValue(-0.1*interval, 0.1*interval), 0.0};
                return_list.push_back(pos);
            }
            break;
        
        default:
            break;
        }
    }
    else
    {
        switch (level)
        {
        case TrafficLevel::L1:
            vehnum = rand() % (4-3+1) + 3;
            interval = 200.0 / double(vehnum - 1);
            for (int i = 0; i < vehnum; i++)
            {
                VehPosition pos = {-575.0 + i*interval + RV->GetValue(-0.1*interval, 0.1*interval), -3.75 - 200.0/sqrt(3.0)};
                if (pos.m_x > -375.0)
                {
                    pos.m_x = -375.0 + sqrt(3.0)*(pos.m_x + 375.0) / 2;
                    pos.m_y = -3.75 - 200.0/sqrt(3.0) + (pos.m_x + 375.0) / 2;
                }
                return_list.push_back(pos);
            }
            break;
        case TrafficLevel::L2:
            vehnum = rand() % (6-4+1) + 4;
            interval = 200.0 / double(vehnum - 1);
            for (int i = 0; i < vehnum; i++)
            {
                VehPosition pos = {-575.0 + i*interval + RV->GetValue(-0.1*interval, 0.1*interval), -3.75 - 200.0/sqrt(3.0)};
                if (pos.m_x > -375.0)
                {
                    pos.m_x = -375.0 + sqrt(3.0)*(pos.m_x + 375.0) / 2;
                    pos.m_y = -3.75 - 200.0/sqrt(3.0) + (pos.m_x + 375.0) / 2;
                }
                return_list.push_back(pos);
            }
            break;
        case TrafficLevel::L3:
            vehnum = rand() % (7-6+1) + 6;
            interval = 200.0 / double(vehnum - 1);
            for (int i = 0; i < vehnum; i++)
            {
                VehPosition pos = {-575.0 + i*interval + RV->GetValue(-0.1*interval, 0.1*interval), -3.75 - 200.0/sqrt(3.0)};
                if (pos.m_x > -375.0)
                {
                    pos.m_x = -375.0 + sqrt(3.0)*(pos.m_x + 375.0) / 2;
                    pos.m_y = -3.75 - 200.0/sqrt(3.0) + (pos.m_x + 375.0) / 2;
                }
                return_list.push_back(pos);
            }
            break;
        
        default:
            break;
        }
    }
    return return_list;
}


void CutInMobilityModel::PrintPositionError(ns3::Ptr<ns3::OutputStreamWrapper> log_stream)
{
    std::cout << "Position Error (100m) : " 
    << CutInMobilityModel::PositionError[0] / CutInMobilityModel::PositionErrorCount[0] 
    << " " << CutInMobilityModel::PositionErrorCount[0] << std::endl;
    std::cout << "Position Error (150m) : " 
    << CutInMobilityModel::PositionError[1] / CutInMobilityModel::PositionErrorCount[1] 
    << " " << CutInMobilityModel::PositionErrorCount[1] << std::endl;
    std::cout << "Position Error (200m) : " 
    << CutInMobilityModel::PositionError[2] / CutInMobilityModel::PositionErrorCount[2] 
    << " " << CutInMobilityModel::PositionErrorCount[2] << std::endl;

    *log_stream->GetStream() << "Position Error (100m) : " 
    << CutInMobilityModel::PositionError[0] / CutInMobilityModel::PositionErrorCount[0] 
    << " " << CutInMobilityModel::PositionErrorCount[0] << std::endl;
    *log_stream->GetStream() << "Position Error (150m) : " 
    << CutInMobilityModel::PositionError[1] / CutInMobilityModel::PositionErrorCount[1] 
    << " " << CutInMobilityModel::PositionErrorCount[1] << std::endl;
    *log_stream->GetStream() << "Position Error (200m) : " 
    << CutInMobilityModel::PositionError[2] / CutInMobilityModel::PositionErrorCount[2] 
    << " " << CutInMobilityModel::PositionErrorCount[2] << std::endl;

}


CutInMobilityModel::CutInMobilityModel()
{
    CutInMobilityModels.push_back(this);
}

CutInMobilityModel::~CutInMobilityModel()
{}

void CutInMobilityModel::DoPassVehState(const VehState& s)
{
    m_vehstates[s.m_ID] = s;
}

CutInMobilityModel::VehState CutInMobilityModel::DoGetVehState()
{
    Time t = Simulator::Now();
    Vector velocity = DoGetVelocity();
    Vector position = DoGetPosition();
    VehState s = {t.GetMilliSeconds(), m_ID, {position.x, position.y}, {velocity.x, velocity.y}};
    return s;
}

void CutInMobilityModel::ScheduleToStart(uint32_t ID, uint32_t start_ms, TrafficLevel level, BelongTo road)
{
    m_ID = ID;
    m_road = road;
    Ptr<ns3::UniformRandomVariable> RV = CreateObject<UniformRandomVariable> ();
    double start_velocity;
    switch (level)
    {
    case TrafficLevel::L1:
        start_velocity = 25.56 + RV->GetValue(-5.0, 5.0);
        break;

    case TrafficLevel::L2:
        start_velocity = 22.56 + RV->GetValue(-5.0, 5.0);
        break;

    case TrafficLevel::L3:
        start_velocity = 18.89 + RV->GetValue(-5.0, 5.0);
        break;
    
    default:
        break;
    }

    Simulator::Schedule(MilliSeconds(start_ms), &CutInMobilityModel::StartToMove, this, start_velocity);
    Simulator::Schedule(MilliSeconds(start_ms + 1), &CutInMobilityModel::UpdateVehStates, this);
    uint32_t offset = rand()%(100 - 2 + 1) + 2;
    Simulator::Schedule(MilliSeconds(start_ms+offset), &CutInMobilityModel::ChangeAction, this);
}

void CutInMobilityModel::StartToMove(double start_velocity)
{
    if (m_road == BelongTo::MAIN_ROAD)
        m_baseVelocity.x = start_velocity;
    else if (m_road == BelongTo::MERGE_ROAD && m_basePosition.x < -375.0)
        m_baseVelocity.x = start_velocity;
    else if (m_road == BelongTo::MERGE_ROAD && m_basePosition.x > -375.0)
        {
            m_baseVelocity.y = start_velocity / 2;
            m_baseVelocity.x = start_velocity * sqrt(3) / 2;
        }
}

void CutInMobilityModel::ChangeAction()
{
    double t = (Simulator::Now () - m_baseTime).GetSeconds ();
    double half_t_square = t*t*0.5;

    Vector v = {m_baseVelocity.x + m_acceleration.x*t, 
                m_baseVelocity.y + m_acceleration.y*t, 
                m_baseVelocity.z + m_acceleration.z*t};
    Vector pos = {m_basePosition.x + m_baseVelocity.x*t + m_acceleration.x*half_t_square,
                  m_basePosition.y + m_baseVelocity.y*t + m_acceleration.y*half_t_square,
                  m_basePosition.z + m_baseVelocity.z*t + m_acceleration.z*half_t_square};

    if ((m_road == BelongTo::MERGE_ROAD) && (m_basePosition.x < -375) && (pos.x > -375.0))
    {
        v.y = v.x/2;
        v.x = v.y * SQRT3;

        pos.x = -375.0 + SQRT3_2*(pos.x + 375.0);
        pos.y = -3.75 - 200.0/SQRT3 + (pos.x + 375.0) / 2;
    }
    else if ((m_road == BelongTo::MERGE_ROAD) && (m_basePosition.x < -175) && (pos.x > -175.0))
    {
        v.x = 2 * v.y;
        v.y = 0.0;

        pos.x = -175.0 + (pos.x + 175.0) / SQRT3_2;
        pos.y = -3.75;
    }

    m_basePosition = pos;
    m_baseVelocity = v;
    m_baseTime = Simulator::Now();

    Action a = GetAction();
    m_acceleration = a.m_acc;

    Simulator::Schedule(MilliSeconds(100), &CutInMobilityModel::ChangeAction, this);
}


CutInMobilityModel::Action
CutInMobilityModel::GetAction()
{
    CalculatePositionError();

    return Action{
        {0.0, 0.0, 0.0},
        0.0
    };
}

void CutInMobilityModel::UpdateVehStates()
{
    for (std::list<CutInMobilityModel*>::iterator ptr = CutInMobilityModels.begin(); ptr != CutInMobilityModels.end(); ptr++)
    {
        m_vehstates[(*ptr)->m_ID] = (*ptr)->DoGetVehState();
    }
}


Vector
CutInMobilityModel::DoGetVelocity (void) const
{
    double t = (Simulator::Now () - m_baseTime).GetSeconds ();
    double half_t_square = t*t*0.5;
    Vector v = {m_baseVelocity.x + m_acceleration.x*t, 
                m_baseVelocity.y + m_acceleration.y*t, 
                m_baseVelocity.z};
    Vector pos = {m_basePosition.x + m_baseVelocity.x*t + m_acceleration.x*half_t_square,
                  m_basePosition.y + m_baseVelocity.y*t + m_acceleration.y*half_t_square,
                  m_basePosition.z};
    if ((m_road == BelongTo::MERGE_ROAD) && (m_basePosition.x < -375) && (pos.x > -375.0))
    {
        v.y = v.x/2;
        v.x = v.y * SQRT3;
    }
    else if ((m_road == BelongTo::MERGE_ROAD) && (m_basePosition.x < -175) && (pos.x > -175.0))
    {
        v.x = 2 * v.y;
        v.y = 0.0;
    }
    return v;
}

inline Vector
CutInMobilityModel::DoGetPosition (void) const
{
    double t = (Simulator::Now () - m_baseTime).GetSeconds ();
    double half_t_square = t*t*0.5;
    Vector pos = {m_basePosition.x + m_baseVelocity.x*t + m_acceleration.x*half_t_square,
                  m_basePosition.y + m_baseVelocity.y*t + m_acceleration.y*half_t_square,
                  m_basePosition.z};
    if ((m_road == BelongTo::MERGE_ROAD) && (m_basePosition.x < -375) && (pos.x > -375.0))
    {
        pos.x = -375.0 + SQRT3_2*(pos.x + 375.0);
        pos.y = -3.75 - 200.0/SQRT3 + (pos.x + 375.0) / 2;
    }
    else if ((m_road == BelongTo::MERGE_ROAD) && (m_basePosition.x < -175) && (pos.x > -175.0))
    {
        pos.x = -175.0 + (pos.x + 175.0) / SQRT3_2;
        pos.y = -3.75;
    }
    return pos;
}

void 
CutInMobilityModel::DoSetPosition (const Vector &position)
{
  m_baseVelocity = DoGetVelocity ();
  m_baseTime = Simulator::Now ();
  m_basePosition = position;
  NotifyCourseChange ();
}

void 
CutInMobilityModel::CalculatePositionError()
{
    Vector position, ego_position;
    double distance, error;
    for (std::list<CutInMobilityModel*>::iterator ptr = CutInMobilityModels.begin(); ptr != CutInMobilityModels.end(); ptr++)
    {
        if ((*ptr)->m_ID == m_ID)
            continue;
        
        position = (*ptr)->DoGetPosition();
        ego_position = DoGetPosition();
        distance = sqrt(pow(position.x-ego_position.x, 2) + 
                        pow(position.y-ego_position.y, 2));
        error = sqrt(pow(position.x-m_vehstates[(*ptr)->m_ID].m_position.m_x, 2) + 
                    pow(position.y-m_vehstates[(*ptr)->m_ID].m_position.m_y, 2));

        // std::cout << m_ID << "  " << error << " " << m_vehstates[(*ptr)->m_ID].m_genTime << std::endl;

        if (distance < 100)
        {
            PositionError[0] += error;
            PositionError[1] += error;
            PositionError[2] += error;
            PositionErrorCount[0]++;
            PositionErrorCount[1]++;
            PositionErrorCount[2]++;
        }
        else if(distance < 150)
        {
            PositionError[1] += error;
            PositionError[2] += error;
            PositionErrorCount[1]++;
            PositionErrorCount[2]++;
        }
        else if (distance < 200)
        {
            PositionError[2] += error;
            PositionErrorCount[2]++;
        }
    }
}

} // namespace ns3
