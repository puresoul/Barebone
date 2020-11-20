# Barebone

OpenVR xinput controller driver for emulating Vive controllers

tracks HMD rotation and position and sets virtual controllers in to field of view. (a little bit similar to riftcat vridge app)

**Words of caution, this driver was my 1st expirience with openvr api, it's worknig but please don't have very high expectation ;-)**

![nXurIGWKoT](https://user-images.githubusercontent.com/2646309/72664664-d1a6c500-3a00-11ea-85a8-59d408d03847.gif)

# Install

simple as copy the barebones folder into your steam drivers folder. Of course you need enable multiple devices in steamvr config


# Controlls

Controllers get two modes single and dual..

+ ltrigger - vive trigger (in dual mode it still controlls last active controller trigger)
+ rtrigger - change mode single/dual

Single:
+ dPad - controlls position movement of active controller 
+ lstick - controlls controller rotation of active controller, click selects 1st controller
+ rstick - controlls vive touchpad of active controller, click selects 2nd controller
+ back - centers active controller
+ lbumper - decrease active controller level
+ rbumper - increase active controller level

Dual:
+ dPad - controlls position movement of all controller 
+ lstick - controlls 1st controller rotation , click switch to controll vive touchpad of that controller
+ rstick - controlls 2nd controller rotation , click switch to controll vive touchpad of that controller
+ back - centers all controller
+ lbumper - decrease all controller level
+ rbumper - increase all controller level

