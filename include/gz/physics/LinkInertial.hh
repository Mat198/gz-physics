/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#ifndef GZ_PHYSICS_LINK_INERTIAL_HH_
#define GZ_PHYSICS_LINK_INERTIAL_HH_

#include <gz/math/Matrix6.hh>

#include <gz/physics/FeatureList.hh>


namespace gz::physics
{

class LinkInertial: public virtual Feature
{
  public: template <typename PolicyT, typename FeaturesT>
  class Link : public virtual Feature::Link<PolicyT, FeaturesT>
  {
    public: void SetInertial(const gz::math::Matrix6d &_inertial);
    public: gz::math::Matrix6d GetInertial() const;
  };

  public: template <typename PolicyT>
  class Implementation : public virtual Feature::Implementation<PolicyT>
  {
    public: virtual void SetLinkInertial(const Identity &_link,
                const gz::math::Matrix6d &_inertial) = 0;
    public: virtual gz::math::Matrix6d GetLinkInertial(
                const Identity &_link) const = 0;
  };
};

template <typename PolicyT, typename FeaturesT>
auto LinkInertial::Link<PolicyT, FeaturesT>::SetInertial(
    const gz::math::Matrix6d &_inertial) -> void
{
  this->template Interface<LinkInertial>()
    ->SetLinkInertial(this->identity, _inertial);
}
template <typename PolicyT, typename FeaturesT>

auto LinkInertial::Link<PolicyT, FeaturesT>::GetInertial(
) const -> gz::math::Matrix6d
{
  return this->template Interface<LinkInertial>()
              ->GetLinkInertial(this->identity);
}

}  // namespace gz::physics

#endif  // GZ_PHYSICS_LINK_INERTIAL_HH_
