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

#include "LinkInertialFeatures.hh"
#include <dart/dynamics/Inertia.hpp>
#include <gz/common/Profiler.hh>

namespace gz::physics::dartsim
{

void LinkInertialFeatures::SetLinkInertial(const Identity &_link,
    const gz::math::Inertiald &_inertial)
{
  GZ_PROFILE("LinkInertialFeatures::SetLinkInertial");
  auto linkInfo = this->ReferenceInterface<LinkInfo>(_link);
  if (!_inertial.MassMatrix().IsValid())
  {
    gzerr << "Invalid inertial for link [" << linkInfo->name
          << "]: mass matrix is invalid. "
          << "The inertial was not changed.\n";
    return;
  }
  AssignInertialToLink(linkInfo, _inertial);
}

gz::math::Inertiald
LinkInertialFeatures::GetLinkInertial(const Identity &_link) const
{
  auto linkInfo = this->ReferenceInterface<LinkInfo>(_link);
  return linkInfo->inertial.value_or(gz::math::Inertiald());
}

}  // namespace gz::physics::dartsim
