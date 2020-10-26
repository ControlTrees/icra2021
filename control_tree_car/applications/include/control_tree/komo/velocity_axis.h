#pragma once

#include <KOMO/komo.h>
#include <velocity.h>

struct VelocityAxis:Feature{

    VelocityAxis(const rai::KinematicWorld& world, const std::string & agent_object)
        : tm_(new TM_Default(TMT_pos, world, agent_object.c_str(), NoVector, NULL, NoVector) )
    {
    }

    virtual rai::String shortTag(const rai::KinematicWorld& G)
    {
      return rai::String("VelocityX");
    }

    virtual void phi(arr& y, arr& J, const rai::KinematicWorld& G) override
    {
        tm_->phi(y, J, G);

        y(1) = 0;
        J(1, 1) = 0;
    }

    virtual uint dim_phi(const rai::KinematicWorld& K) override
    {
      return tm_->dim_phi(K);
    }

private:
    std::shared_ptr<TM_Default> tm_;
};
