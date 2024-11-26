`default_nettype none
`include "USB.svh"
`include "USBPkg.pkg"

// Wrapper for USB bus states. Notice that enum Z can only be driven, not read
typedef enum logic [1:0]
  {BS_J = 2'b10, BS_K = 2'b01, BS_SE0 = 2'b00, BS_SE1 = 2'b11, BS_NC = 2'bzz}
  bus_state_t;

/* Mimics the USB Host so that we can send different kinds of transactions to
our devices */
module USBHost (
  USBWires wires,
  input logic clock, reset_n);

  pkt_t currentPkt, finishedPkt;

  // Sender side signals
  logic ldSendSync, ldSendPid, ldSendPay, ldSendAddr, ldSendEndp, bitStuffReg, 
        nrziEnReg, currentBit, ldBitEncode, pktSent, readyForStuff, stopPkt, 
        readyForPkt, ldSendCRC5, ldSendCRC16, startCRC5, startCRC16, startPass, 
        endPkt, sendBit, finishPkt, startEncode, twiddleWires, ldZero, 
        ldOne;
  
  // Receiver side signals
  logic ldReceiveSync, ldReceivePid, ldReceivePay, ldReceiveAddr, ldReceiveEndp,
        ldReceiveCRC5, ldReceiveCRC16, takeBit, receiveCRC5, 
        receiveCRC16, storeSync, storePid, storePay, storeAddr, storeEndp, 
        receiveBit, startSkip, currentDecodeBit, ldDecodedZero, ldDecodedOne, 
        currentUnstuffBit, doneSkip, bitUnStuffReg, dpDmReg, nrziDeReg, 
        sendSyncBits, sendPidBits, sendPayBits, sendAddrBits, sendEndpBits, 
        sendCRC5, sendCRC16, incUnstuffCounter, clrUnstuffCounter, incorrectCRC;
  logic [4:0] unstuffCounter;

  // Dp/Dm signals
  logic endSeq, readDP, readDM, donePkt, sendPkt, receivePkt, startDecode, 
        doneReceive, ldDpDm0, ldDpDm1, incZeroCounter, clrZeroCounter,
        wiresTouched;
  logic [2:0] countZeroes;
  tri1 dpWire;
  tri0 dmWire;

  // Read/Write signals
  logic read, write, readyForTransaction, outSent, dataSent, inSent,
        dataReceived, notReceived, dataSent, notSent, sendData, sendIN, sendOUT, 
        correct, finish, getDataIn, sendDataOut;

  // Protocal handler signals
  logic restart, ldIN, ldOUT, ldOUTMem, ldNAK, ldACK, ldMemIn, ldMemOut, 
        ldDataOut, ldMem, incRetryNAK, incRetryTime, incTimeout;
  logic [3:0] retryTime, retryNAK;
  logic [7:0] timeoutCount;
  logic [15:0] mem;
  logic [63:0] dataOut;

  logic [63:0] currentField;
  logic [6:0] lengthField;
  logic [4:0] finishedSendCRC5, finishedReceiveCRC5;
  logic [15:0] finishedSendCRC16, finishedReceiveCRC16;

  // Calls the OUT packet to be sent to ADDR 63 and ENDP 4
  task prelabRequest();
    // pktSent = 1;
    // currentPkt.pid = PID_OUT;
    // currentPkt.addr = `DEVICE_ADDR;
    // currentPkt.endp = `ADDR_ENDP;
    // wait(stopPkt);
  endtask : prelabRequest

  task readData
  // Host sends mempage to thumb drive using a READ (OUT->DATA0->IN->DATA0)
  // transaction, and then receives data from it. This task should return both the
  // data and the transaction status, successful or unsuccessful, to the caller.
  ( input logic [15:0] mempage, // Page to write
    output logic [63:0] data, // Vector of bytes to write
    output logic success);

    mem <= mempage;
    read <= 1;

    wait(finish);
    
    read <= 0;
    data <= finishedPkt.payload;
    success <= correct;
    @(posedge clock);
    @(posedge clock);

  endtask : readData

  task writeData
  // Host sends mempage to thumb drive using a WRITE (OUT->DATA0->OUT->DATA0)
  // transaction, and then sends data to it. This task should return the
  // transaction status, successful or unsuccessful, to the caller.
  ( input logic [15:0] mempage, // Page to write
    input logic [63:0] data, // Vector of bytes to write
    output logic success);
    
    mem <= mempage;
    dataOut <= data;
    write <= 1;

    wait(finish);

    write <= 0;
    success <= correct;
    @(posedge clock);
    @(posedge clock);

  endtask : writeData

  // Communicates with the OS to perform read/writes in the USB
  readWriteFSM rdWr(.clock, .reset_n, .read, .write, .readyForTransaction, 
                    .outSent, .dataSent, .inSent, .dataReceived, .notReceived, 
                    .dataSent, .notSent, .sendData, .sendIN, .sendOUT, .correct,
                    .finish, .getDataIn, .sendDataOut);

  protocolHandler handler(.clock, .reset_n, .sendOUT, .sendIN, .readyForPkt, 
                          .donePkt, .sendData, .wiresTouched, .doneReceive, 
                          .incorrectCRC, .retryTime, .retryNAK, .timeoutCount,
                          .decodedPkt(finishedPkt), .readyForTransaction, 
                          .mem, .dataOut, .pktSent, .sendPkt, .outSent, 
                          .notSent, .incRetryNAK, .incRetryTime, .restart, 
                          .dataSent, .receivePkt, .inSent, .incTimeout, 
                          .notReceived, .dataReceived, .ldIN, .ldOUT, .ldNAK, 
                          .ldACK, .ldMem, .ldDataOut, .ldOUTMem, .sendDataOut);

  // Current packet being sent
  always_ff @(posedge clock) begin
    if (~reset_n) begin
      currentPkt.pid <= '0;
      currentPkt.payload <= '0;
      currentPkt.addr <= '0;
      currentPkt.endp <= '0;
    end
    else if (ldOUTMem) begin
      currentPkt.pid <= PID_OUT;
      currentPkt.payload <= '0;
      currentPkt.addr <= `DEVICE_ADDR;
      currentPkt.endp <= `ADDR_ENDP;
    end
    else if (ldIN) begin
      currentPkt.pid <= PID_IN;
      currentPkt.payload <= '0;
      currentPkt.addr <= `DEVICE_ADDR;
      currentPkt.endp <= `DATA_ENDP;
    end
    else if (ldOUT) begin
      currentPkt.pid <= PID_OUT;
      currentPkt.payload <= '0;
      currentPkt.addr <= `DEVICE_ADDR;
      currentPkt.endp <= `DATA_ENDP;
    end
    else if (ldNAK) begin
      currentPkt.pid <= PID_NAK;
      currentPkt.payload <= '0;
      currentPkt.addr <= '0;
      currentPkt.endp <= '0;
    end
    else if (ldACK) begin
      currentPkt.pid <= PID_ACK;
      currentPkt.payload <= '0;
      currentPkt.addr <= '0;
      currentPkt.endp <= '0;
    end
    else if (ldDataOut) begin
      currentPkt.pid <= PID_DATA0;
      currentPkt.payload <= dataOut;
      currentPkt.addr <= '0;
      currentPkt.endp <= '0;
    end
    else if (ldMem) begin
      currentPkt.pid <= PID_DATA0;
      currentPkt.payload[63:48] <= mem;
      currentPkt.addr <= '0;
      currentPkt.endp <= '0;
    end
  end

  // Timeout counter
  always_ff @(posedge clock) begin
    if (~reset_n)
      timeoutCount <= '0;
    else if (incTimeout)
      timeoutCount <= timeoutCount + 1;
    else if (restart)
      timeoutCount <= '0;
  end

  // Timeout retry counter
  always_ff @(posedge clock) begin
    if (~reset_n)
      retryTime <= '0;
    else if (incRetryTime)
      retryTime <= retryTime + 1;
    else if (finish)
      retryTime <= '0;
  end

  // NAK retry counter
  always_ff @(posedge clock) begin
    if (~reset_n)
      retryNAK <= '0;
    else if (incRetryNAK)
      retryNAK <= retryNAK + 1;
    else if (finish)
      retryNAK <= '0;
  end

  /***                            SENDER SIDE                               ***/

  /* Takes the packet field and sends a bit stream to the CRC, Bit Stuffer, NZRI
  and the DP/DM */
  bitStreamEncoding encode(.clock, .reset_n, .pktSent, .readyForStuff, .stopPkt, 
                           .lengthField, .currentPkt, .readyForPkt, 
                           .ldSendSync, .ldSendPid, .ldSendPay, .ldSendAddr, 
                           .ldSendEndp, .ldSendCRC5, .ldSendCRC16, 
                           .startCRC5, .startCRC16, .startPass, .endPkt,
                           .sendSyncBits, .sendPidBits, .sendPayBits, 
                           .sendAddrBits, .sendEndpBits, .sendCRC5, .sendCRC16,
                           .sendBit);

  // Performs the CRC5 on non DATA0 packets
  CRC5 sendErrorDetect5(.clock, .reset_n, .enable(startCRC5), .currentBit, 
                        .finishedCRC5(finishedSendCRC5), .stopPkt);

  // Performs the CRC16 on the DATA0 packets
  CRC16 sendErrorDetect16(.clock, .reset_n, .enable(startCRC16), .currentBit, 
                          .finishedCRC16(finishedSendCRC16), .stopPkt);

  // Takes the bit stream given and inserts a 0 for every 6 conesecutive 1s
  bitStuffing stuff(.clock, .reset_n, .startPass, 
                    .currentStuffBit(currentBit), .endPkt, .finishPkt, 
                    .readyForStuff, .startEncode, .stopPkt, 
                    .startStuffing(startPass));

  // Takes the bit stream and performs NRZI encoding on it
  nrziEncoding nrziEn(.clock, .reset_n, .startEncode, .donePkt, 
                      .currentEncodeBit(bitStuffReg), .twiddleWires, .endSeq, 
                      .finishPkt, .ldZero, .ldOne);

  /* Register to choose with field of the packet we are currently looking at and
  the length of that field */
  always_ff @(posedge clock) begin
    if (~reset_n) begin
      currentField <= '0;
      lengthField <= '0;
    end
    else if (ldSendSync || ldReceiveSync) begin
      currentField <= `SYNC;
      lengthField <= `SYNC_BITS;
    end
    else if (ldSendPid || ldReceivePid) begin
      currentField <= {~currentPkt.pid, currentPkt.pid};
      lengthField <= (`PID_BITS * 2);
    end
    else if (ldSendPay || ldReceivePay) begin
      currentField <= currentPkt.payload;
      lengthField <= `PAYLOAD_BITS;
    end
    else if (ldSendAddr || ldReceiveAddr) begin
      currentField <= currentPkt.addr;
      lengthField <= `ADDR_BITS;
    end
    else if (ldSendEndp || ldReceiveEndp) begin
      currentField <= currentPkt.endp;
      lengthField <= `ENDP_BITS;
    end
    else if (ldSendCRC5) begin
      lengthField <= `CRC5_BITS;
      currentField <= finishedSendCRC5;
    end
    else if (ldReceiveCRC5) begin
      lengthField <= `CRC5_BITS;
      currentField <= finishedReceiveCRC5;
    end
    else if (ldSendCRC16) begin
      lengthField <= `CRC16_BITS;
      currentField <= finishedSendCRC16;
    end
    else if (ldReceiveCRC16) begin
      lengthField <= `CRC16_BITS;
      currentField <= finishedReceiveCRC16;
    end
    else if (sendBit || receiveBit)
      lengthField <= lengthField - 1;
  end

  // Current bit for CRCs
  always_comb begin
    if (sendSyncBits)
      currentBit = currentField[lengthField - 1];
    else if (sendPidBits)
      currentBit = currentField[((`PID_BITS * 2) - lengthField)];
    else if (sendPayBits)
      currentBit = currentField[(`PAYLOAD_BITS - lengthField)];
    else if (sendAddrBits)
      currentBit = currentField[(`ADDR_BITS - lengthField)];
    else if (sendEndpBits)
      currentBit = currentField[(`ENDP_BITS - lengthField)];
    else if (sendCRC5)
      currentBit = finishedSendCRC5[lengthField - 1];
    else if (sendCRC16)
      currentBit = finishedSendCRC16[lengthField - 1];
  end

  // Bit Stuff Register
  always_ff @(posedge clock) begin
    if (~reset_n)
      bitStuffReg <= 0;
    else if (readyForStuff)
      bitStuffReg <= currentBit;
    else if (~readyForStuff)
      bitStuffReg <= 0;
  end

  // NRZI Encode Register
  always_ff @(posedge clock) begin
    if (~reset_n)
      nrziEnReg <= 0;
    else if (ldZero)
      nrziEnReg <= 0;
    else if (ldOne)
      nrziEnReg <= 1;
  end

  /***                        END OF SENDER SIDE                            ***/


  /***                          RECEIVER SIDE                               ***/

  /* Takes the bit stream and puts together the corresponding packet */
  bitStreamDecoding decode(.clock, .reset_n, .takeBit, .finishedPkt, 
                           .lengthField, .ldReceiveSync, .ldReceivePid, 
                           .ldReceivePay, .ldReceiveAddr, .ldReceiveEndp, 
                           .ldReceiveCRC5, .ldReceiveCRC16, .receiveCRC5, 
                           .receiveCRC16, .storeSync, .storePid, .storePay, 
                           .storeAddr, .storeEndp, .receiveBit, 
                           .finishedReceiveCRC5, .finishedReceiveCRC16,
                           .incorrectCRC);
  
  // Performs the CRC5 on non DATA0 packets
  CRC5 receiveErrorDetect5(.clock, .reset_n, .enable(receiveCRC5), 
                           .currentBit(bitUnStuffReg),
                           .finishedCRC5(finishedReceiveCRC5), 
                           .stopPkt(receivePkt));

  // Performs the CRC16 on the DATA0 packets
  CRC16 receiveErrorDetect16(.clock, .reset_n, .enable(receiveCRC16), 
                             .currentBit(bitUnStuffReg), 
                             .finishedCRC16(finishedReceiveCRC16), 
                             .stopPkt(receivePkt));
  
  // Takes the bit stream and performs NRZI decoding on it
  nrziDecoding nrziDe(.clock, .reset_n, .startDecode, .doneReceive, 
                      .currentDecodeBit(dpDmReg), .ldDecodedZero, .ldDecodedOne, 
                      .startSkip);

  // Takes the bit stream given and removes a 0 for every 6 conesecutive 1s
  bitUnstuffing unstuff(.clock, .reset_n, .startSkip, 
                        .currentUnstuffBit(nrziDeReg), .skipBits(unstuffCounter),
                        .doneReceive, .takeBit, .incUnstuffCounter, 
                        .clrUnstuffCounter);

  // NRZI Decode Register
  always_ff @(posedge clock) begin
    if (~reset_n)
      nrziDeReg <= 0;
    else if (ldDecodedZero)
      nrziDeReg <= 0;
    else if (ldDecodedOne)
      nrziDeReg <= 1;
  end

  // Bit Unstuff Register
  always_ff @(posedge clock) begin
    if (~reset_n)
      bitUnStuffReg <= 0;
    else if (takeBit)
      bitUnStuffReg <= nrziDeReg;
  end

  // Final Packet Register
  always_ff @(posedge clock) begin
    if (~reset_n) begin
      finishedPkt.pid <= '0;
      finishedPkt.payload <= '0;
      finishedPkt.addr <= '0;
      finishedPkt.endp <= '0;
    end
    else if (storePid)
      finishedPkt.pid[((`PID_BITS * 2) - lengthField)] <= bitUnStuffReg;
    else if (storePay)
      finishedPkt.payload[(`PAYLOAD_BITS - lengthField)] <= bitUnStuffReg;
    else if (storeAddr)
      finishedPkt.addr[(`ADDR_BITS - lengthField)] <= bitUnStuffReg;
    else if (storeEndp)
      finishedPkt.endp[(`ENDP_BITS - lengthField)] <= bitUnStuffReg;
  end

  // Counter to help the bit unstuffer skip the PID bits
  always_ff @(posedge clock) begin
    if (~reset_n)
      unstuffCounter <= '0;
    else if (incUnstuffCounter)
      unstuffCounter <= unstuffCounter + 1;
    else if (clrUnstuffCounter)
      unstuffCounter <= '0;
  end
                      
  /***                       END OF RECEIVER SIDE                           ***/


  // Sets D+ and D- wires to their correct values based on the bit stream
  dpDMFSM setWires(.clock, .reset_n, .twiddleWires, .endSeq, .readDP, .readDM,
                   .currentLevels(nrziEnReg), .donePkt, .dpWire, 
                   .dmWire, .sendPkt, .receivePkt, .startDecode, .doneReceive,
                   .ldDpDm0, .ldDpDm1, .countZeroes, .incZeroCounter,
                   .clrZeroCounter, .restart, .wiresTouched);

  assign wires.DP = (twiddleWires) ? dpWire : 1'bz; // A tristate driver
  assign wires.DM = (twiddleWires) ? dmWire : 1'bz; // Another tristate driver

  assign readDP = wires.DP; // getting access to DP and DM
  assign readDM = wires.DM;

  // DPDM Register
  always_ff @(posedge clock) begin
    if (~reset_n)
      dpDmReg <= 0;
    else if (ldDpDm0)
      dpDmReg <= 0;
    else if (ldDpDm1)
      dpDmReg <= 1;
  end

  // Counts zeroes to check when SYNC is received on dp/dm wires
  always_ff @(posedge clock) begin
    if (~reset_n)
      countZeroes <= '0;
    else if (incZeroCounter)
      countZeroes <= countZeroes + 1;
    else if (clrZeroCounter)
      countZeroes <= '0;
  end

endmodule : USBHost

/* Mimics the CRC5 hardware in order to calculate the residue for the non DATA0 
packets */
module CRC5
  (input logic clock, reset_n, enable, currentBit, stopPkt,
  output logic [4:0] finishedCRC5);

  logic [2:0] firstThreeReg;
  logic [1:0] lastTwoReg;
  logic lastTwoXor, firstThreeXor;

  assign lastTwoXor = (firstThreeReg[2] ^ currentBit);
  assign firstThreeXor = (lastTwoReg[1] ^ lastTwoXor);
  assign finishedCRC5 = ~({firstThreeReg, lastTwoReg});

  // Creates the last two flip flops in the CRC5 hardware
  always_ff @(posedge clock) begin
    if (~reset_n)
      lastTwoReg <= '1;
    else if (stopPkt)
      lastTwoReg <= '1;
    else if (enable)
      lastTwoReg <= {lastTwoReg[0], lastTwoXor};
  end

  // Creates the first three flip flops in the CRC5 hardware
  always_ff @(posedge clock) begin
    if (~reset_n)
      firstThreeReg <= '1;
    else if (stopPkt)
      firstThreeReg <= '1;
    else if (enable)
      firstThreeReg <= {firstThreeReg[1:0], firstThreeXor};
  end

endmodule: CRC5

/* Mimics the CRC16 hardware in order to calculate the residue for the DATA0 
packets */
module CRC16
  (input logic clock, reset_n, enable, currentBit, stopPkt,
  output logic [15:0] finishedCRC16);

  logic firstOneReg;
  logic [12:0] middleThirteenReg;
  logic [1:0] lastTwoReg;
  logic firstOneXor, middleThirteenXor, lastTwoXor;

  assign firstOneXor = (middleThirteenReg[12] ^ lastTwoXor);
  assign middleThirteenXor = (lastTwoReg[1] ^ lastTwoXor);
  assign lastTwoXor = (firstOneReg ^ currentBit);
  assign finishedCRC16 = ~({firstOneReg, middleThirteenReg, lastTwoReg});

  // Creates the last two flip flops in the CRC16 hardware
  always_ff @(posedge clock) begin
    if (~reset_n)
      lastTwoReg <= '1;
    else if (stopPkt)
      lastTwoReg <= '1;
    else if (enable)
      lastTwoReg <= {lastTwoReg[0], lastTwoXor};
  end

  // Creates the middle thirteen flip flops in the CRC16 hardware
  always_ff @(posedge clock) begin
    if (~reset_n)
      middleThirteenReg <= '1;
    else if (stopPkt)
      middleThirteenReg <= '1;
    else if (enable)
      middleThirteenReg <= {middleThirteenReg[11:0], middleThirteenXor};
  end

  // Creates the first flip flop in the CRC16 hardware
  always_ff @(posedge clock) begin
    if (~reset_n)
      firstOneReg <= '1;
    else if (stopPkt)
      firstOneReg <= '1;
    else if (enable)
      firstOneReg <= firstOneXor;
  end

endmodule: CRC16

/* Takes the packet field given and turns it into the appropiate bit stream for
the CRCs, Bit Stuffer, NRZI, and DP/DM. Each state in the FSM is made for the
field given */
module bitStreamEncoding
  (input logic clock, reset_n, pktSent, readyForStuff, stopPkt,
  input logic [6:0] lengthField,
  input pkt_t currentPkt,
  output logic readyForPkt, ldSendSync, ldSendPid, ldSendPay, ldSendAddr, 
               ldSendEndp, ldSendCRC5, ldSendCRC16, startCRC5, startCRC16, 
               startPass, endPkt, sendSyncBits, sendPidBits, sendPayBits, 
               sendAddrBits, sendEndpBits, sendCRC5, sendCRC16, sendBit);

  enum logic [3:0] {START, SENDSYNC, SENDPID, SENDEOP, SENDPAY, SENDADDR, 
                    SENDCRC16, SENDENDP, SENDCRC5, LAST} 
                    currentState, nextState;

  // Next State and Output Logic
  always_comb begin
    readyForPkt = 0; ldSendSync = 0; ldSendPid = 0; ldSendPay = 0; 
    ldSendAddr = 0; ldSendEndp = 0; ldSendCRC5 = 0; ldSendCRC16 = 0; 
    startCRC5 = 0; startCRC16 = 0; startPass = 0; endPkt = 0; sendSyncBits = 0; 
    sendPidBits = 0; sendPayBits = 0; sendAddrBits = 0; sendEndpBits = 0; 
    sendCRC5 = 0; sendCRC16 = 0; sendBit = 0;
    unique case (currentState)
      START: begin 
        if (pktSent) begin
          nextState = SENDSYNC;
          ldSendSync = 1;
          readyForPkt = 1;
          startPass = 1;
        end
        else begin
          nextState = START;  
          readyForPkt = 1;
        end
      end
      SENDSYNC: begin
        if (~readyForStuff)
          nextState = SENDSYNC;
        else if (readyForStuff && (lengthField != 1)) begin
          nextState = SENDSYNC;
          startPass = 1;
          sendSyncBits = 1;
          sendBit = 1;
        end
        else if (readyForStuff && (lengthField == 1)) begin
          nextState = SENDPID;
          ldSendPid = 1;
          startPass = 1;
          sendSyncBits = 1;
        end
      end
      SENDPID: begin
        if (~readyForStuff)
          nextState = SENDPID;
        else if (readyForStuff && (lengthField != 1)) begin
          nextState = SENDPID;
          startPass = 1;
          sendPidBits = 1;
          sendBit = 1;
        end
        else if (readyForStuff && (lengthField == 1) && 
                ((currentPkt.pid == PID_ACK) || (currentPkt.pid == PID_NAK)))
        begin
          nextState = SENDEOP;
          startPass = 1;
          sendPidBits = 1;
        end
        else if (readyForStuff && (lengthField == 1) && 
                (currentPkt.pid == PID_DATA0)) begin
          nextState = SENDPAY;
          startPass = 1;
          ldSendPay = 1;
          sendPidBits = 1;
        end
        else if (readyForStuff && (lengthField == 1) && 
                ((currentPkt.pid == PID_OUT) || (currentPkt.pid == PID_IN)))
        begin
          nextState = SENDADDR;
          startPass = 1;
          ldSendAddr = 1;
          sendPidBits = 1;
        end
      end
      SENDEOP: begin
        nextState = LAST;
      end
      SENDPAY: begin
        if (~readyForStuff)
          nextState = SENDPAY;
        else if (readyForStuff && (lengthField != 1)) begin
          nextState = SENDPAY;
          sendPayBits = 1;
          sendBit = 1;
          startCRC16 = 1;
        end
        else if (readyForStuff && (lengthField == 1)) begin
          nextState = SENDCRC16;
          sendPayBits = 1;
          startCRC16 = 1;
          ldSendCRC16 = 1;
        end
      end
      SENDADDR: begin
        if (~readyForStuff) begin
          nextState = SENDADDR;
        end
        else if (readyForStuff && (lengthField != 1)) begin
          nextState = SENDADDR;
          sendAddrBits = 1; 
          sendBit = 1;
          startCRC5 = 1;
        end
        else if (readyForStuff && (lengthField == 1)) begin
          nextState = SENDENDP;
          ldSendEndp = 1;
          sendAddrBits = 1;
          startCRC5 = 1;
        end
      end
      SENDCRC16: begin
        if (~readyForStuff)
          nextState = SENDCRC16;
        else if (readyForStuff && (lengthField != 1)) begin
          nextState = SENDCRC16;
          sendBit = 1;
          sendCRC16 = 1;
        end
        else if (readyForStuff && (lengthField == 1)) begin
          nextState = SENDEOP;
          sendCRC16 = 1;
        end
      end
      SENDENDP: begin
        if (~readyForStuff)
          nextState = SENDENDP;
        else if (readyForStuff && (lengthField != 1)) begin
          nextState = SENDENDP;
          sendEndpBits = 1;
          sendBit = 1;
          startCRC5 = 1;
        end
        else if (readyForStuff && (lengthField == 1)) begin
          nextState = SENDCRC5;
          sendEndpBits = 1;
          startCRC5 = 1;
          ldSendCRC5 = 1;
        end
      end
      SENDCRC5: begin
        if (~readyForStuff)
          nextState = SENDCRC5;
        else if (readyForStuff && (lengthField != 1)) begin
          nextState = SENDCRC5;
          sendBit = 1;
          sendCRC5 = 1;
        end
        else if (readyForStuff && (lengthField == 1)) begin
          nextState = SENDEOP;
          sendCRC5 = 1;
        end
      end
      LAST: begin
        if (~stopPkt) begin
          nextState = LAST;
          endPkt = 1;
        end
        else if (stopPkt) begin
          nextState = START;
          readyForPkt = 1;
        end
      end
    endcase
  end

  // State Register
  always_ff @(posedge clock) begin
    if (~reset_n)
      currentState <= START;
    else
      currentState <= nextState;
  end

endmodule: bitStreamEncoding

/* Takes the bit stream and puts a 0 in it after every 6 consecutive 1s. Skips
the fields that do not need to be bit stuffed */
module bitStuffing
  (input logic clock, reset_n, startPass, currentStuffBit, endPkt, finishPkt,
               startStuffing, 
   output logic readyForStuff, startEncode, stopPkt);

  enum logic [3:0] {START, STOP, SEEN1, SEEN2, SEEN3, SEEN4, SEEN5, SEEN6, 
                    SEND0} currentState, nextState;

  // Next State and Output Logic
  always_comb begin
    readyForStuff = 1; startEncode = 0; stopPkt = 0;
    unique case (currentState)
      START: begin
        if (startStuffing)
          nextState = SEEN1;
        else if (~startStuffing)
          nextState = START;
      end
      SEEN1: begin
        if (endPkt) begin
          nextState = STOP;
          startEncode = 1;
        end
        else if (startPass | (~startPass && (currentStuffBit != 1))) begin
          nextState = SEEN1;
          startEncode = 1;
        end
        else if (~startPass && (currentStuffBit == 1)) begin
          nextState = SEEN2;
          startEncode = 1;
        end
      end
      SEEN2: begin
        if (endPkt) begin
          nextState = STOP;
          startEncode = 1;
        end
        else if ((currentStuffBit != 1)) begin
          nextState = SEEN1;
          startEncode = 1;
        end
        else if (currentStuffBit == 1) begin
          nextState = SEEN3;
          startEncode = 1;
        end
      end
      SEEN3: begin
        if (endPkt) begin
          nextState = STOP;
          startEncode = 1;
        end
        else if ((currentStuffBit != 1)) begin
          nextState = SEEN1;
          startEncode = 1;
        end
        else if (currentStuffBit == 1) begin
          nextState = SEEN4;
          startEncode = 1;
        end
      end
      SEEN4: begin
        if (endPkt) begin
          nextState = STOP;
          startEncode = 1;
        end
        else if ((currentStuffBit != 1)) begin
          nextState = SEEN1;
          startEncode = 1;
        end
        else if (currentStuffBit == 1) begin
          nextState = SEEN5;
          startEncode = 1;
        end
      end
      SEEN5: begin
        if (endPkt) begin
          nextState = STOP;
          startEncode = 1;
        end
        else if ((currentStuffBit != 1)) begin
          nextState = SEEN1;
          startEncode = 1;
        end
        else if (currentStuffBit == 1) begin
          nextState = SEEN6;
          startEncode = 1;
        end
      end
      SEEN6: begin
        if (endPkt) begin
          nextState = STOP;
          startEncode = 1;
        end
        else if ((currentStuffBit != 1)) begin
          nextState = SEEN1;
          startEncode = 1;
        end
        else if (currentStuffBit == 1) begin
          nextState = SEND0;
          startEncode = 1;
        end
      end
      SEND0: begin
        if (endPkt) begin
          nextState = STOP;
          startEncode = 1;
          readyForStuff = 0;
        end
        else begin
          nextState = SEEN1;
          startEncode = 1;
          readyForStuff = 0;
        end
      end
      STOP: begin
        if (finishPkt) begin
          nextState = START;
          stopPkt = 1;
        end
        else 
          nextState = STOP;
      end
    endcase
  end


  // State Register
  always_ff @(posedge clock) begin
    if (~reset_n)
      currentState <= START;
    else
      currentState <= nextState;
  end

endmodule: bitStuffing

// Takes the given bit stream and encodes it using the NRZI encoding
module nrziEncoding
  (input logic clock, reset_n, startEncode, donePkt, currentEncodeBit,
  output logic twiddleWires, endSeq, finishPkt, ldZero, ldOne);

  enum logic [2:0] {START, ENCODE, PREV0, PREV1, PASS} currentState, nextState;

  // Next State and Output Logic
  always_comb begin
    twiddleWires = 0; endSeq = 0; finishPkt = 0; ldOne = 0; ldZero = 0;
    unique case(currentState)
      START: begin
        if (startEncode) begin
          nextState = ENCODE;
        end
        else if (~startEncode) begin
          nextState = START;
        end
      end
      ENCODE: begin
        if (currentEncodeBit == 1) begin
          nextState = PREV1;
          ldOne = 1;
        end
        else if (currentEncodeBit == 0) begin
          nextState = PREV0;
          ldZero = 1;
        end
      end
      PREV0: begin
        if (~startEncode) begin
          nextState = PASS;
          endSeq = 1;
          twiddleWires = 1;
        end
        else if (currentEncodeBit == 1) begin
          nextState = PREV0;
          twiddleWires = 1;
          ldZero = 1;
        end
        else if (currentEncodeBit == 0) begin
          nextState = PREV1;
          twiddleWires = 1;
          ldOne = 1;
        end
      end
      PREV1: begin
        if (~startEncode) begin
          nextState = PASS;
          endSeq = 1;
          twiddleWires = 1;
        end
        else if (currentEncodeBit == 1) begin
          nextState = PREV1;
          twiddleWires = 1;
          ldOne = 1;
        end
        else if (currentEncodeBit == 0) begin
          nextState = PREV0;
          twiddleWires = 1;
          ldZero = 1;
        end
      end
      PASS: begin
        if (donePkt) begin
          nextState = START;
          finishPkt = 1;
        end
        else if (~donePkt) begin
          nextState = PASS;
          twiddleWires = 1;
        end
      end
    endcase
  end

  // State Register
  always_ff @(posedge clock) begin
    if (~reset_n)
      currentState <= START;
    else
      currentState <= nextState;
  end

endmodule: nrziEncoding

/* Takes the bit stream given and sets the D+ and D- wires depending on if the
the bitstream had a K or J in it */
module dpDMFSM
  (input logic clock, reset_n, twiddleWires, endSeq, currentLevels, sendPkt, 
               receivePkt, readDP, readDM, restart, 
  input logic [2:0] countZeroes,
  output logic donePkt, startDecode, doneReceive, dpWire, dmWire, ldDpDm0, 
               ldDpDm1, incZeroCounter, clrZeroCounter, wiresTouched);

  enum logic [4:0] {SETWIRES, SETX1, SETX2, SETJ, START, CHECKWIRES, SAWX1, 
                    SAWX2, SAWJ, WAITSYNC} currentState, nextState;

  // Next State and Output Logic
  always_comb begin
    donePkt = 0; dpWire = 1'bz; dmWire = 1'bz; startDecode = 0; doneReceive = 0;
    ldDpDm0 = 0; ldDpDm1 = 0; incZeroCounter = 0; clrZeroCounter = 0;
    wiresTouched = 0;
    unique case (currentState)
      START: begin
        if (sendPkt)
          nextState = SETWIRES;
        else if (receivePkt)
          nextState = CHECKWIRES;
        else
          nextState = START;
      end
      SETWIRES: begin
        if (endSeq) begin
          nextState = SETX1;
          dpWire = 0;
          dmWire = 0;
        end
        // else if (endSeq && (currentLevels == 1)) begin
        //   nextState = SETX1;
        //   dpWire = 1;
        //   dmWire = 0;
        // end
        else if (~twiddleWires) begin
          nextState = SETWIRES;
          dpWire = 1'bz;
          dmWire = 1'bz;
        end
        else if (twiddleWires && (currentLevels == 0)) begin
          nextState = SETWIRES;
          dpWire = 0;
          dmWire = 1;
        end
        else if (twiddleWires && (currentLevels == 1)) begin
          nextState = SETWIRES;
          dpWire = 1;
          dmWire = 0;
        end
      end
      SETX1: begin
        nextState = SETJ;
        dpWire = 0;
        dmWire = 0;
      end
      // SETX2: begin
      //   nextState = SETJ;
      //   dpWire = 0;
      //   dmWire = 0;
      // end
      SETJ: begin
        nextState = START;
        dpWire = 1;
        dmWire = 0;
        donePkt = 1;
      end 
      // WAITSYNC: begin
      //   if (restart)
      //     nextState = START;
      //   else if (readDP == 0 && readDM == 1) begin
      //     nextState = WAITSYNC;
      //     incZeroCounter = 1;
      //     wiresTouched = 1;
      //   end
      //   else  if ((readDP == 1 && readDM == 0) && countZeroes != 7) begin
      //     nextState = WAITSYNC;
      //     clrZeroCounter = 1;
      //   end
      //   else if ((readDP == 1 && readDM == 0) && countZeroes == 7) begin
      //     nextState = CHECKWIRES;
      //     clrZeroCounter = 1;
      //   end
      //   else begin
      //     nextState = WAITSYNC;
      //     clrZeroCounter = 1;
      //   end
      // end
      CHECKWIRES: begin
        if (restart)
          nextState = START;
        else if ((readDP == 1 && readDM == 0) && (countZeroes == 1)) begin
          nextState = CHECKWIRES;
          startDecode = 1;
          ldDpDm1 = 1;
          wiresTouched = 1;
        end
        else if ((readDP == 1 && readDM == 0) && (countZeroes == 0)) begin
          nextState = CHECKWIRES;
        end
        else if ((readDP == 0 && readDM == 1) && (countZeroes == 0)) begin
          nextState = CHECKWIRES;
          startDecode = 1;
          ldDpDm0 = 1;
          wiresTouched = 1;
          incZeroCounter = 1;
        end
        else if ((readDP == 0 && readDM == 1) && (countZeroes == 1)) begin
          nextState = CHECKWIRES;
          startDecode = 1;
          ldDpDm0 = 1;
          wiresTouched = 1;
        end
        else if (readDP == 0 && readDM == 0) begin
          nextState = SAWX1;
          clrZeroCounter = 1;
        end
        else
          nextState = CHECKWIRES;
      end
      SAWX1: begin
        nextState = SAWX2;
      end
      SAWX2: begin
        nextState = SAWJ;
      end
      SAWJ: begin
        nextState = START;
        doneReceive = 1;
      end
    endcase
  end

  // State Register
  always_ff @(posedge clock) begin
    if (~reset_n)
      currentState <= START;
    else
      currentState <= nextState;
  end
endmodule: dpDMFSM

// Takes the given bit stream and decodes it using the NRZI decoding
module nrziDecoding
  (input logic clock, reset_n, startDecode, doneReceive, currentDecodeBit,
  output logic ldDecodedZero, ldDecodedOne, startSkip);

  enum logic [2:0] {START, DECODE, PREV0, PREV1} currentState, nextState;

  // Next State and Output Logic
  always_comb begin
    ldDecodedZero = 0; ldDecodedOne = 0; startSkip = 0;
    unique case(currentState)
      START: begin
        if (startDecode) begin
          nextState = DECODE;
        end
        else if (~startDecode) begin
          nextState = START;
        end
      end
      DECODE: begin
        if (currentDecodeBit == 1) begin
          nextState = PREV1;
          ldDecodedOne = 1;
          startSkip = 1;
        end
        else if (currentDecodeBit == 0) begin
          nextState = PREV0;
          ldDecodedZero = 1;
          startSkip = 1;
        end
      end
      PREV0: begin
        if (doneReceive) begin
          nextState = START;
        end
        else if (currentDecodeBit == 1) begin
          nextState = PREV1;
          ldDecodedZero = 1;
        end
        else if (currentDecodeBit == 0) begin
          nextState = PREV0;
          ldDecodedOne = 1;
        end
      end
      PREV1: begin
        if (doneReceive) begin
          nextState = START;
        end
        else if (currentDecodeBit == 1) begin
          nextState = PREV1;
          ldDecodedOne = 1;
        end
        else if (currentDecodeBit == 0) begin
          nextState = PREV0;
          ldDecodedZero = 1;
        end
      end
    endcase
  end

  // State Register
  always_ff @(posedge clock) begin
    if (~reset_n)
      currentState <= START;
    else
      currentState <= nextState;
  end

endmodule: nrziDecoding


/* Takes the bit stream and ignores the 0 after every 6 consecutive 1s. Skips
the fields that do not need to be bit unstuffed */
module bitUnstuffing
  (input logic clock, reset_n, startSkip, currentUnstuffBit, 
               doneReceive, 
   input logic [4:0] skipBits,
   output logic takeBit, incUnstuffCounter, clrUnstuffCounter);

  enum logic [3:0] {START, PASS, SEEN1, SEEN2, SEEN3, SEEN4, SEEN5, SEEN6, HOLD} 
                    currentState, nextState;

  // Next State and Output Logic
  always_comb begin
    takeBit = 1; incUnstuffCounter = 0; clrUnstuffCounter = 0;
    unique case (currentState)
      START: begin
        if (startSkip) begin
          nextState = PASS;
          takeBit = 0;
        end
        else if (~startSkip) begin
          nextState = START;
          takeBit = 0;
        end
      end
      PASS: begin
        if (skipBits == 16) begin
          nextState = SEEN1;
          clrUnstuffCounter = 1;
          takeBit = 1;
        end
        else begin
          nextState = PASS;
          incUnstuffCounter = 1;
          takeBit = 1;
        end
      end
      SEEN1: begin
        if (doneReceive) begin
          nextState = START;
        end
        else if ((currentUnstuffBit != 1)) begin
          nextState = SEEN1;
          takeBit = 1;
        end
        else if ((currentUnstuffBit == 1)) begin
          nextState = SEEN2;
          takeBit = 1;
        end
      end
      SEEN2: begin
        if (doneReceive) begin
          nextState = START;
        end
        else if ((currentUnstuffBit != 1)) begin
          nextState = SEEN1;
        end
        else if (currentUnstuffBit == 1) begin
          nextState = SEEN3;
        end
      end
      SEEN3: begin
        if (doneReceive) begin
          nextState = START;
        end
        else if ((currentUnstuffBit != 1)) begin
          nextState = SEEN1;
        end
        else if (currentUnstuffBit == 1) begin
          nextState = SEEN4;
        end
      end
      SEEN4: begin
        if (doneReceive) begin
          nextState = START;
        end
        else if ((currentUnstuffBit != 1)) begin
          nextState = SEEN1;
        end
        else if (currentUnstuffBit == 1) begin
          nextState = SEEN5;
        end
      end
      SEEN5: begin
        if (doneReceive) begin
          nextState = START;
        end
        else if ((currentUnstuffBit != 1)) begin
          nextState = SEEN1;
        end
        else if (currentUnstuffBit == 1) begin
          nextState = SEEN6;
        end
      end
      SEEN6: begin
        if (doneReceive) begin
          nextState = START;
        end
        else if ((currentUnstuffBit != 1)) begin
          nextState = SEEN1;
        end
        else if (currentUnstuffBit == 1) begin
          nextState = HOLD;
        end
      end
      HOLD: begin
        if (doneReceive) begin
          nextState = START;
          takeBit = 0;
        end
        else begin
          nextState = SEEN1;
          takeBit = 0;
        end
      end
    endcase
  end


  // State Register
  always_ff @(posedge clock) begin
    if (~reset_n)
      currentState <= START;
    else
      currentState <= nextState;
  end

endmodule: bitUnstuffing


/* Takes the bit stream given and turns it into the appropiate packet. Each 
   state in the FSM is made for the field given */
module bitStreamDecoding
  (input logic clock, reset_n, takeBit,
  input pkt_t finishedPkt,
  input logic [6:0] lengthField, 
  input logic [4:0] finishedReceiveCRC5,
  input logic [15:0] finishedReceiveCRC16,
  output logic ldReceiveSync, ldReceivePid, ldReceivePay, ldReceiveAddr, 
               ldReceiveEndp, ldReceiveCRC5, ldReceiveCRC16, receiveCRC5, 
               receiveCRC16, storeSync, storePid, storePay, storeAddr, 
               storeEndp, receiveBit, incorrectCRC);

  enum logic [3:0] {START, GETSYNC, GETPID, GETEOP, GETPAY, GETADDR, 
                    GETCRC16, GETENDP, GETCRC5} 
                    currentState, nextState;

  // Next State and Output Logic
  always_comb begin
    ldReceiveSync = 0; ldReceivePid = 0; ldReceivePay = 0; ldReceiveAddr = 0; 
    ldReceiveEndp = 0; ldReceiveCRC5 = 0; ldReceiveCRC16 = 0; receiveCRC5 = 0; 
    receiveCRC16 = 0; storeSync = 0; storePid = 0; storePay = 0; storeAddr = 0; 
    storeEndp = 0; receiveBit = 0; incorrectCRC = 0;
    unique case (currentState)
      START: begin 
        if (takeBit) begin
          nextState = GETSYNC;
          ldReceiveSync = 1;
        end
        else begin
          nextState = START;  
        end
      end
      GETSYNC: begin
        if (~takeBit)
          nextState = GETSYNC;
        else if (takeBit && (lengthField != 1)) begin
          nextState = GETSYNC;
          storeSync = 1;
          receiveBit = 1; 
        end
        else if (takeBit && (lengthField == 1)) begin
          nextState = GETPID;
          ldReceivePid = 1;
          storeSync = 1;
        end
      end
      GETPID: begin
        if (~takeBit)
          nextState = GETPID;
        else if (takeBit && (lengthField > 4)) begin
          nextState = GETPID;
          storePid = 1;
          receiveBit = 1;
        end
        else if (takeBit && (lengthField == 1) && 
                ((finishedPkt.pid == PID_ACK) || (finishedPkt.pid == PID_NAK)))
        begin
          nextState = GETEOP;
        end
        else if (takeBit && (lengthField == 1) && 
                (finishedPkt.pid == PID_DATA0)) begin
          nextState = GETPAY;
          ldReceivePay = 1;
        end
        else if (takeBit && (lengthField == 1) && 
                ((finishedPkt.pid == PID_OUT) || (finishedPkt.pid == PID_IN)))
        begin
          nextState = GETADDR;
          ldReceiveAddr = 1;
        end
        else begin
          nextState = GETPID;
          receiveBit = 1;
        end
      end
      GETEOP: begin
        nextState = START;
      end
      GETPAY: begin
        if (~takeBit)
          nextState = GETPAY;
        else if (takeBit && (lengthField != 1)) begin
          nextState = GETPAY;
          storePay = 1;
          receiveCRC16 = 1;
          receiveBit = 1;
        end
        else if (takeBit && (lengthField == 1)) begin
          nextState = GETCRC16;
          storePay = 1;
          ldReceiveCRC16 = 1;
          receiveCRC16 = 1;
        end
      end
      GETADDR: begin
        if (~takeBit) begin
          nextState = GETADDR;
        end
        else if (takeBit && (lengthField != 1)) begin
          nextState = GETADDR;
          storeAddr = 1;
          receiveCRC5 = 1;
          receiveBit = 1;
        end
        else if (takeBit && (lengthField == 1)) begin
          nextState = GETENDP;
          storeAddr = 1;
          ldReceiveEndp = 1;
          receiveCRC5 = 1;
        end
      end
      GETCRC16: begin
        if (~takeBit)
          nextState = GETCRC16;
        else if (takeBit && (lengthField > 1)) begin
          nextState = GETCRC16;
          receiveCRC16 = 1;
          receiveBit = 1;
        end
        else if (takeBit && (lengthField == 1)) begin
          nextState = GETCRC16;
          receiveCRC16 = 1;
          receiveBit = 1;
        end
        else if (takeBit && (lengthField == 0) && 
                (finishedReceiveCRC16 == `CRC16_RESIDUE)) 
        begin
          nextState = GETEOP;
          incorrectCRC = 0;
        end
        else if (takeBit && (lengthField == 0) && 
                (finishedReceiveCRC16 != `CRC16_RESIDUE)) 
        begin
          nextState = GETEOP;
          incorrectCRC = 1;
        end
      end
      GETENDP: begin
        if (~takeBit)
          nextState = GETENDP;
        else if (takeBit && (lengthField != 1)) begin
          nextState = GETENDP;
          storeEndp = 1;
          receiveCRC5 = 1;
          receiveBit = 1;
        end
        else if (takeBit && (lengthField == 1)) begin
          nextState = GETCRC5;
          storeEndp = 1;
          ldReceiveCRC5 = 1;
          receiveCRC5 = 1;
        end
      end
      GETCRC5: begin
        if (~takeBit)
          nextState = GETCRC5;
        else if (takeBit && (lengthField != 1)) begin
          nextState = GETCRC5;
          receiveCRC5 = 1;
          receiveBit = 1;
        end
        else if (takeBit && (lengthField == 1)) begin
          nextState = GETEOP;
          receiveCRC5 = 1;
          receiveBit = 1;
        end
        else if (takeBit && (lengthField == 0) && 
                (finishedReceiveCRC5 == `CRC5_RESIDUE)) 
        begin
          nextState = GETEOP;
          incorrectCRC = 0;
        end
        else if (takeBit && (lengthField == 0) && 
                (finishedReceiveCRC5 != `CRC5_RESIDUE)) 
        begin
          nextState = GETEOP;
          incorrectCRC = 1;
        end
      end
    endcase
  end

  // State Register
  always_ff @(posedge clock) begin
    if (~reset_n)
      currentState <= START;
    else
      currentState <= nextState;
  end

endmodule: bitStreamDecoding

module readWriteFSM
  (input logic clock, reset_n, read, write, readyForTransaction, outSent, 
               dataSent, inSent, dataReceived, notReceived, notSent,
  output logic sendData, sendIN, sendOUT, correct, finish, getDataIn, 
               sendDataOut);

  enum logic [2:0] {READY, OUTMEM, SENDMEM, INDATA, OUTDATA, GETDATA, 
                    SENDDATA} currentState, nextState;

  // Next state and output logic
  always_comb begin
    sendData = 0; sendIN = 0; sendOUT = 0; correct = 0; finish = 0;
    getDataIn = 0; sendDataOut = 0;
    unique case(currentState)
      READY: begin
        if (readyForTransaction && (read || write)) begin
          nextState = OUTMEM;
          sendOUT = 1;
        end
        else
          nextState = READY;
      end
      OUTMEM: begin
        if (outSent) begin
          nextState = SENDMEM;
          sendData = 1;
        end
        else begin
          nextState = OUTMEM;
        end
      end
      SENDMEM: begin
        if (notSent) begin
          nextState = READY;
          correct = 0;
          finish = 1;
        end
        if (dataSent & read) begin
          nextState = INDATA;
          sendIN = 1;
        end
        else if (dataSent & write) begin
          nextState = OUTDATA;
          sendOUT = 1;
        end
        else begin
          nextState = SENDMEM;
          sendData = 1;
        end
      end
      INDATA: begin
        if (inSent) begin
          nextState = GETDATA;
          getDataIn = 1;
        end
        else begin
          nextState = INDATA;
          sendIN = 1;
        end
      end
      OUTDATA: begin
        if (outSent) begin
          nextState = SENDDATA;
          sendDataOut = 1;
        end
        else begin
          nextState = OUTDATA;
          sendOUT = 1;
        end
      end
      GETDATA: begin
        if (notReceived) begin
          nextState = READY;
          correct = 0;
          finish = 1;
        end
        else if (dataReceived) begin
          nextState = READY;
          correct = 1;
          finish = 1;
        end
        else begin
          nextState = GETDATA;
          getDataIn = 1;
        end
      end
      SENDDATA: begin
        if (notSent) begin
          nextState = READY;
          correct = 0;
          finish = 1;
        end
        else if (dataSent) begin
          nextState = READY;
          correct = 1;
          finish = 1;
        end
        else begin
          nextState = SENDDATA;
          sendDataOut = 1;
        end
      end
    endcase
  end

  // State register
  always_ff @(posedge clock) begin
    if (~reset_n)
      currentState <= READY;
    else
      currentState <= nextState;
  end

endmodule: readWriteFSM

module protocolHandler
  (input logic clock, reset_n, sendOUT, sendIN, readyForPkt, donePkt, sendData, 
               wiresTouched, doneReceive, incorrectCRC, sendDataOut,
   input logic [3:0] retryTime, retryNAK,
   input logic [7:0] timeoutCount,
   input logic [15:0] mem,
   input logic [63:0] dataOut,
   input pkt_t decodedPkt,
   output logic readyForTransaction, pktSent, sendPkt, outSent, notSent,
                incRetryNAK, incRetryTime, restart, dataSent, receivePkt, inSent,
                incTimeout, notReceived, dataReceived, ldIN, ldOUT, ldNAK, 
                ldACK, ldMem, ldDataOut, ldOUTMem);

  enum logic [4:0] {START, WAITOUT, CREATEMEM, WAITMEM, CHECKMEM, INOROUT, 
                    HOLDIN, HOLDOUT, LOOKDIN, SENDNAK, WAITDIN, CHECKDIN, 
                    SENDACK, CREATEDOUT, WAITDOUT, CHECKDOUT} currentState,
                    nextState;

  always_comb begin
    readyForTransaction = 0; pktSent = 0; sendPkt = 0; outSent = 0;
    notSent = 0; incRetryNAK = 0; incRetryTime = 0; restart = 0; dataSent = 0; 
    receivePkt = 0; inSent = 0; incTimeout = 0; notReceived = 0; dataReceived = 0;
    ldIN = 0; ldOUT = 0; ldNAK = 0; ldACK = 0; ldMem = 0; ldDataOut = 0;
    ldOUTMem = 0;
    unique case(currentState)
      START: begin
        if (readyForPkt && sendOUT) begin
          nextState = WAITOUT;
          pktSent = 1;
          ldOUTMem = 1;
          sendPkt = 1;
          readyForTransaction = 1;
        end
        else begin
          nextState = START;
          readyForTransaction = 1;
          //pktSent = 1;
        end
      end
      WAITOUT: begin
        if (donePkt) begin
          nextState = CREATEMEM;
          outSent = 1;
        end
        else
          nextState = WAITOUT;
      end
      CREATEMEM: begin
        if ((retryTime == `TX_RETRIES) || (retryNAK == `TX_RETRIES)) begin
          nextState = START;
          notSent = 1;
        end
        else if (readyForPkt && sendData) begin
          nextState = WAITMEM;
          pktSent = 1;
          ldMem = 1;
          sendPkt = 1;
        end
        else begin
          nextState = CREATEMEM;
        end
      end
      WAITMEM: begin
        if (donePkt) begin
          nextState = CHECKMEM;
          receivePkt = 1;
        end
        else
          nextState = WAITMEM;
      end
      CHECKMEM: begin
        if (timeoutCount == `TIMEOUT) begin
          nextState = CREATEMEM;
          incRetryTime = 1;
          restart = 1;
        end
        else if (doneReceive && decodedPkt.pid == PID_NAK) begin
          nextState = CREATEMEM;
          incRetryNAK = 1;
          restart = 1;
        end
        else if (doneReceive && decodedPkt.pid == PID_ACK) begin
          nextState = INOROUT;
          dataSent = 1;
        end
        else if (wiresTouched) begin
          nextState = CHECKMEM;
          receivePkt = 1;
        end
        else begin
          nextState = CHECKMEM;
          incTimeout = 1;
          receivePkt = 1;
        end
      end
      INOROUT: begin
        if (readyForPkt && sendOUT) begin
          nextState = HOLDOUT;
          pktSent = 1;
          ldOUT = 1;
          sendPkt = 1;
        end
        else if (readyForPkt && sendIN) begin
          nextState = HOLDIN;
          pktSent = 1;
          ldIN = 1;
          sendPkt = 1;
        end
        else begin
          nextState = INOROUT;
        end
      end
      HOLDIN: begin
        if (donePkt) begin
          nextState = LOOKDIN;
          inSent = 1;
          receivePkt = 1;
        end
        else
          nextState = HOLDIN;
      end
      HOLDOUT: begin
        if (donePkt) begin
          nextState = CREATEDOUT;
          outSent = 1;
        end
        else
          nextState = HOLDOUT;
      end
      LOOKDIN: begin
        if ((retryTime == `TX_RETRIES) || (retryNAK == `TX_RETRIES)) begin
          nextState = START;
          notReceived = 1;
        end
        else if (timeoutCount == `TIMEOUT) begin
          nextState = SENDNAK;
          incRetryTime = 1;
          restart = 1;
        end
        else if (wiresTouched)
          nextState = WAITDIN;
        else begin
          nextState = LOOKDIN;
          incTimeout = 1;
          receivePkt = 1;
        end
      end
      SENDNAK: begin
        if (donePkt) begin
          nextState = LOOKDIN;
        end
        else begin
          nextState = SENDNAK;
          pktSent = 1;
          ldNAK = 1;
          sendPkt = 1;
        end
      end
      WAITDIN: begin
        if (doneReceive)
          nextState = CHECKDIN;
        else
          nextState = WAITDIN;
      end
      CHECKDIN: begin
        if (incorrectCRC) begin
          nextState = SENDNAK;
          incRetryNAK = 1;
          restart = 1;
        end
        else begin
          nextState = SENDACK;
          pktSent = 1;
          ldACK = 1;
          sendPkt = 1;
        end
      end
      SENDACK: begin
        if (donePkt) begin
          nextState = START;
          dataReceived = 1;
        end
        else begin
          nextState = SENDACK;
        end
      end
      CREATEDOUT: begin
        if ((retryTime == `TX_RETRIES) || (retryNAK == `TX_RETRIES)) begin
          nextState = START;
          notSent = 1;
        end
        else if (readyForPkt && sendDataOut) begin
          nextState = WAITDOUT;
          pktSent = 1;
          ldDataOut = 1;
          sendPkt = 1;
        end
        else
          nextState = CREATEDOUT;
      end
      WAITDOUT: begin
        if (donePkt) begin
          nextState = CHECKDOUT;
          receivePkt = 1;
        end
        else begin
          nextState = WAITDOUT;
        end
      end
      CHECKDOUT: begin
        if (timeoutCount == `TIMEOUT) begin
          nextState = CREATEDOUT;
          incRetryTime = 1;
          restart = 1;
        end
        else if (doneReceive && decodedPkt.pid == PID_NAK) begin
          nextState = CREATEDOUT;
          incRetryNAK = 1;
          restart = 1;
        end
        else if (doneReceive && decodedPkt.pid == PID_ACK) begin
          nextState = START;
          dataSent = 1;
        end
        else if (wiresTouched)
          nextState = CHECKDOUT;
        else begin
          nextState = CHECKDOUT;
          incTimeout = 1;
          receivePkt = 1;
        end
      end
    endcase
  end

  // State register
  always_ff @(posedge clock) begin
    if (~reset_n)
      currentState <= START;
    else
      currentState <= nextState;
  end

endmodule: protocolHandler


