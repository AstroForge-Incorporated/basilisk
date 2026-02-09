/*
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

 */

#ifndef FSSCONFIGLOGSIMMSG_H
#define FSSCONFIGLOGSIMMSG_H

//!@brief FSS configuration message log message
/*! This message is the outpout of each FSS device to log all the configuration and
    measurement states.
 */
typedef struct
    //@cond DOXYGEN_IGNORE
    FSSConfigLogMsgPayload
//@endcond
{
    double r_B[3] = {0};     //!< [m] sensor position vector in the spacecraft, "B", body frame
    double fov;              //!< [rad] field of view (boresight to edge)
    double solar_irradiance; //!< [-] solar irradiance measurement without perturbations
    double senNoiseBound;      //!< sensor noise bounds
    double dcm_cf2b[9];      //!< DCM from component frame to spacecraft body frame
    int FSSGroupID = 0;      //!< [] Group ID if the FSS is part of a cluster
} FSSConfigLogMsgPayload;

#endif /* FSSCONFIGLOGSIMMSG_H */
