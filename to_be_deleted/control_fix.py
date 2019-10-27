'''
    # Function to read present angle for motor #id, output radians
    # Fix some error in library for read4ByteTxRx
    def read4ByteTxRx(self, port, dxl_id, address):
        data, result, error = self.readTxRx(port, dxl_id, address, 4)
        data_read = DXL_MAKEDWORD(DXL_MAKEWORD(data[0], data[1]),
                                    DXL_MAKEWORD(data[2], data[3])) if (result == COMM_SUCCESS) else 0
        return data_read, result, error

    def readTxRx(self, port, dxl_id, address, length):
        PKT_PARAMETER0 = 5
        txpacket = [0] * 8
        data = []

        if dxl_id >= BROADCAST_ID:
            return data, COMM_NOT_AVAILABLE, 0

        txpacket[self.PKT_ID] = dxl_id
        txpacket[self.PKT_LENGTH] = 4
        txpacket[self.PKT_INSTRUCTION] = INST_READ
        txpacket[self.PKT_PARAMETER0 + 0] = address
        txpacket[self.PKT_PARAMETER0 + 1] = length

        rxpacket, result, error = self.txRxPacket(port, txpacket)
        if result == COMM_SUCCESS:
            error = rxpacket[self.PKT_ERROR]

            data.extend(rxpacket[PKT_PARAMETER0: PKT_PARAMETER0 + length])

        return data, result, error

    def txRxPacket(self, port, txpacket):
        rxpacket = None
        error = 0

        # tx packet
        result = self.packetHandler.txPacket(port, txpacket)
        if result != COMM_SUCCESS:
            return rxpacket, result, error

        # (Instruction == BulkRead) == this function is not available.
        if txpacket[self.PKT_INSTRUCTION] == INST_BULK_READ:
            result = COMM_NOT_AVAILABLE

        # (ID == Broadcast ID) == no need to wait for status packet or not available
        if (txpacket[self.PKT_ID] == BROADCAST_ID):
            port.is_using = False
            return rxpacket, result, error

        # set packet timeout
        if txpacket[self.PKT_INSTRUCTION] == INST_READ:
            port.setPacketTimeout(txpacket[self.PKT_PARAMETER0 + 1] + 6)
        else:
            port.setPacketTimeout(6)  # HEADER0 HEADER1 ID LENGTH ERROR CHECKSUM

        # rx packet
        while True:
            rxpacket, result = self.rxPacket(port)
            if result != COMM_SUCCESS or txpacket[self.PKT_ID] == rxpacket[self.PKT_ID]:
                break

        if result == COMM_SUCCESS and txpacket[self.PKT_ID] == rxpacket[self.PKT_ID]:
            error = rxpacket[self.PKT_ERROR]

        return rxpacket, result, error

    def rxPacket(self, port):
        rxpacket = []

        result = COMM_TX_FAIL
        checksum = 0
        rx_length = 0
        wait_length = 10  # minimum length (HEADER0 HEADER1 ID LENGTH ERROR CHKSUM)
        # FIX: CHANGED 6 TO 10 for read_pos

        while True:
            rxpacket.extend(port.readPort(wait_length - rx_length))
            rx_length = len(rxpacket)
            if rx_length >= wait_length:
                # find packet header
                for idx in range(0, (rx_length - 1)):
                    if (rxpacket[idx] == 0xFF) and (rxpacket[idx + 1] == 0xFF):
                        break

                if idx == 0:  # found at the beginning of the packet
                    if (rxpacket[self.PKT_ID] > 0xFD) or (rxpacket[self.PKT_LENGTH] > RXPACKET_MAX_LEN) or (
                            rxpacket[self.PKT_ERROR] > 0x7F):
                        # unavailable ID or unavailable Length or unavailable Error
                        # remove the first byte in the packet
                        del rxpacket[0]
                        rx_length -= 1
                        continue

                    # re-calculate the exact length of the rx packet
                    if wait_length != (rxpacket[self.PKT_LENGTH] + self.PKT_LENGTH + 1):
                        wait_length = rxpacket[self.PKT_LENGTH] + self.PKT_LENGTH + 1
                        continue

                    if rx_length < wait_length:
                        # check timeout
                        if port.isPacketTimeout():
                            if rx_length == 0:
                                result = COMM_RX_TIMEOUT
                            else:
                                result = COMM_RX_CORRUPT
                            break
                        else:
                            continue

                    # calculate checksum
                    for i in range(2, wait_length - 1):  # except header, checksum
                        checksum += rxpacket[i]
                    checksum = ~checksum & 0xFF

                    # verify checksum
                    if rxpacket[wait_length - 1] == checksum:
                        result = COMM_SUCCESS
                    else:
                        result = COMM_RX_CORRUPT
                    break

                else:
                    # remove unnecessary packets
                    del rxpacket[0: idx]
                    rx_length -= idx

            else:
                # check timeout
                if port.isPacketTimeout():
                    if rx_length == 0:
                        result = COMM_RX_TIMEOUT
                    else:
                        result = COMM_RX_CORRUPT
                    break

        port.is_using = False

        #print "[RxPacket] %r" % rxpacket

        return rxpacket, result
    # end fix

    def read_pos(self, id):
        dxl_present_position, dxl_comm_result, dxl_error = self.read4ByteTxRx(self.portHandler, id, self.ADDR_MX_PRESENT_POSITION)
        if dxl_comm_result != COMM_SUCCESS:
            return self.read_pos(id)
        elif dxl_error != 0:
            return self.read_pos(id)
        else:
            if(dxl_present_position > 1023 or dxl_present_position<0):
                return self.read_pos(id)
            return dxl_present_position, dxl_comm_result, dxl_error
'''