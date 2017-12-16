/**
* @file GroundContactState.h
*/

#ifndef _GROUNDCONTACTSTATE_H_
#define _GROUNDCONTACTSTATE_H_

namespace Robot{
/**
* @class GroundContactState
* Describes whether we got contact with ground or not.
*/
class GroundContactState
{
public:
  /** Default constructor. */
  GroundContactState() : contactSafe(true), contact(true), noContactSafe(false) {}

  bool contactSafe; /**< Whether we got contact with ground or not. */
  bool contact;
  bool noContactSafe; /** Whether we got no contact with ground or not */
};
}
#endif //GroundContactState_H
