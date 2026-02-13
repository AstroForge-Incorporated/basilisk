#include "dummyAttDet.h"

#include "architecture/utilities/linearAlgebra.h"

void DummyAttitudeDet::UpdateState(uint64_t current_sim_nanos) {
    // Read spacecraft message
    SCStatesMsgPayload scStatesInBuffer = this->spacecraftStatesInMsg();

    // Write out message
    NavAttMsgPayload navAttOutBuffer;
    v3Copy(scStatesInBuffer.sigma_BN, navAttOutBuffer.sigma_BN);
    v3Copy(scStatesInBuffer.omega_BN_B, navAttOutBuffer.omega_BN_B);

    this->navAttOutMsg.write(&navAttOutBuffer, moduleID, current_sim_nanos);
}
