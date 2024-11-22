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
        endPkt, sendBit, finishPkt, startEncode, donePkt, twiddleWires, endSeq, 
        ldZero, ldOne, readDP, readDM, send, receive, startDecode;
  
  // Receiver side signals
  logic ldReceiveSync, ldReceivePid, ldReceivePay, ldReceiveAddr, ldReceiveEndp,
        ldReceiveCRC5, ldReceiveCRC16, doneReceive, takeBit, receiveCRC5, 
        receiveCRC16, storeSync, storePid, storePay, storeAddr, storeEndp, 
        receiveBit, startSkip, currentDecodeBit, ldDecodedZero, ldDecodedOne, 
        currentUnstuffBit, doneSkip, bitUnStuffReg, dpDmReg, ldDpDm0, ldDpDm1, 
        nrziDeReg, sendSyncBits, sendPidBits, sendPayBits, sendAddrBits, 
        sendEndpBits, sendCRC5, sendCRC16;

  logic [63:0] currentField;
  logic [6:0] lengthField;
  logic [4:0] finishedSendCRC5, finishedReceiveCRC5;
  logic [15:0] finishedSendCRC16, finishedReceiveCRC16;
  tri1 dpWire;
  tri0 dmWire;

  // Calls the OUT packet to be sent to ADDR 63 and ENDP 4
  task prelabRequest();
    pktSent = 1;
    currentPkt.pid = PID_OUT;
    currentPkt.addr = `DEVICE_ADDR;
    currentPkt.endp = `ADDR_ENDP;
    send = 1;
    wait(stopPkt);
  endtask : prelabRequest

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
      currentBit = currentField[lengthField - 1];
    else if (sendCRC16)
      currentBit = currentField[lengthField - 1];
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

  // DPDM Register
  always_ff @(posedge clock) begin
    if (~reset_n)
      dpDmReg <= 0;
    else if (ldDpDm0)
      dpDmReg <= 0;
    else if (ldDpDm1)
      dpDmReg <= 1;
  end

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

  /***                        END OF SENDER SIDE                            ***/


  /***                          RECEIVER SIDE                               ***/

  /* Takes the bit stream and puts together the corresponding packet */
  bitStreamDecoding decode(.clock, .reset_n, .takeBit, .finishedPkt, 
                           .lengthField, .ldReceiveSync, .ldReceivePid, 
                           .ldReceivePay, .ldReceiveAddr, .ldReceiveEndp, 
                           .ldReceiveCRC5, .ldReceiveCRC16, .receiveCRC5, 
                           .receiveCRC16, .storeSync, .storePid, .storePay, 
                           .storeAddr,.storeEndp, .receiveBit, 
                           .finishedReceiveCRC5, .finishedReceiveCRC16);
  
  // Performs the CRC5 on non DATA0 packets
  CRC5 receiveErrorDetect5(.clock, .reset_n, .enable(startCRC5), .currentBit,
                           .finishedCRC5(finishedReceiveCRC5), .stopPkt());

  // Performs the CRC16 on the DATA0 packets
  CRC16 receiveErrorDetect16(.clock, .reset_n, .enable(startCRC16), .currentBit, 
                             .finishedCRC16(finishedReceiveCRC16), .stopPkt());
  
  // Takes the bit stream and performs NRZI decoding on it
  nrziDecoding nrziDe(.clock, .reset_n, .startDecode, .doneReceive, 
                      .currentDecodeBit(dpDmReg), .ldDecodedZero, .ldDecodedOne, 
                      .startSkip);

  // Takes the bit stream given and removes a 0 for every 6 conesecutive 1s
  bitUnstuffing unstuff(.clock, .reset_n, .startSkip, 
                        .currentUnstuffBit(nrziDeReg), .doneSkip, .doneReceive, 
                        .takeBit);
                      
  /***                       END OF RECEIVER SIDE                           ***/


  // Sets D+ and D- wires to their correct values based on the bit stream
  dpDMFSM setWires(.clock, .reset_n, .twiddleWires, .endSeq, .readDP, .readDM,
                   .currentLevels(nrziEnReg), .donePkt, .dpWire, 
                   .dmWire, .send, .receive, .startDecode, .doneReceive,
                   .ldDpDm0, .ldDpDm1);

  assign wires.DP = (twiddleWires) ? dpWire : 1'bz; // A tristate driver
  assign wires.DM = (twiddleWires) ? dmWire : 1'bz; // Another tristate driver

  assign readDP = wires.DP; // getting access to DP and DM
  assign readDM = wires.DM;

  task readData
  // Host sends mempage to thumb drive using a READ (OUT->DATA0->IN->DATA0)
  // transaction, and then receives data from it. This task should return both the
  // data and the transaction status, successful or unsuccessful, to the caller.
  ( input logic [15:0] mempage, // Page to write
    output logic [63:0] data, // Vector of bytes to write
    output logic success);

    data = 64'h0;
    success = 1'b0;

  endtask : readData

  task writeData
  // Host sends mempage to thumb drive using a WRITE (OUT->DATA0->OUT->DATA0)
  // transaction, and then sends data to it. This task should return the
  // transaction status, successful or unsuccessful, to the caller.
  ( input logic [15:0] mempage, // Page to write
    input logic [63:0] data, // Vector of bytes to write
    output logic success);

    success = 1'b0;

  endtask : writeData

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
  assign finishedCRC5 = ~({((firstThreeReg << 1) | firstThreeXor), 
                          ((lastTwoReg << 1) | lastTwoXor)});

  /* Creates a register to hold the final residue value once the CRC5 is done
  calculating */
  // always_ff @(posedge clock) begin
  //   if (~reset_n)
  //     finishedCRC5 <= '1;
  //   else if (stopPkt)
  //     finishedCRC5 <= '1;
  //   else if (enable)
  //     finishedCRC5 <= ~({((firstThreeReg << 1) | firstThreeXor), 
  //                      ((lastTwoReg << 1) | lastTwoXor)});
  // end

  // Creates the last two flip flops in the CRC5 hardware
  always_ff @(posedge clock) begin
    if (~reset_n)
      lastTwoReg <= '1;
    else if (stopPkt)
      lastTwoReg <= '1;
    else if (enable)
      lastTwoReg <= ((lastTwoReg << 1) | lastTwoXor);
  end

  // Creates the first three flip flops in the CRC5 hardware
  always_ff @(posedge clock) begin
    if (~reset_n)
      firstThreeReg <= '1;
    else if (stopPkt)
      firstThreeReg <= '1;
    else if (enable)
      firstThreeReg <= ((firstThreeReg << 1) | firstThreeXor);
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

  /* Creates a register to hold the final residue value once the CRC16 is done
  calculating */ 
  always_ff @(posedge clock) begin
    if (~reset_n)
      finishedCRC16 <= '1;
    else if (stopPkt)
      finishedCRC16 <= '1;
    else if (enable)
      finishedCRC16 <= ~({((firstOneReg << 1) | firstOneXor), 
                         ((middleThirteenReg << 1) | middleThirteenXor), 
                         ((lastTwoReg << 1) | lastTwoXor)});
  end

  // Creates the last two flip flops in the CRC16 hardware
  always_ff @(posedge clock) begin
    if (~reset_n)
      lastTwoReg <= '1;
    else if (stopPkt)
      lastTwoReg <= '1;
    else if (enable)
      lastTwoReg <= ((lastTwoReg << 1) | lastTwoXor);
  end

  // Creates the middle thirteen flip flops in the CRC16 hardware
  always_ff @(posedge clock) begin
    if (~reset_n)
      middleThirteenReg <= '1;
    else if (stopPkt)
      middleThirteenReg <= '1;
    else if (enable)
      middleThirteenReg <= ((middleThirteenReg << 1) | middleThirteenXor);
  end

  // Creates the first flip flop in the CRC16 hardware
  always_ff @(posedge clock) begin
    if (~reset_n)
      firstOneReg <= '1;
    else if (stopPkt)
      firstOneReg <= '1;
    else if (enable)
      firstOneReg <= ((firstOneReg << 1) | firstOneXor);
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
          readyForPkt = 0;
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
        else if (stopPkt)
          nextState = START;
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

  enum logic [2:0] {START, SEEN1, SEEN2, SEEN3, SEEN4, SEEN5, SEEN6, SEND0} 
                    currentState, nextState;

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
        if (finishPkt) begin
          nextState = START;
          stopPkt = 1;
        end
        else if (endPkt) begin
          nextState = SEEN1;
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
        if ((currentStuffBit != 1) | endPkt) begin
          nextState = SEEN1;
          startEncode = 1;
        end
        else if (currentStuffBit == 1) begin
          nextState = SEEN3;
          startEncode = 1;
        end
      end
      SEEN3: begin
        if ((currentStuffBit != 1) | endPkt) begin
          nextState = SEEN1;
          startEncode = 1;
        end
        else if (currentStuffBit == 1) begin
          nextState = SEEN4;
          startEncode = 1;
        end
      end
      SEEN4: begin
        if ((currentStuffBit != 1) | endPkt) begin
          nextState = SEEN1;
          startEncode = 1;
        end
        else if (currentStuffBit == 1) begin
          nextState = SEEN5;
          startEncode = 1;
        end
      end
      SEEN5: begin
        if ((currentStuffBit != 1) | endPkt) begin
          nextState = SEEN1;
          startEncode = 1;
        end
        else if (currentStuffBit == 1) begin
          nextState = SEEN6;
          startEncode = 1;
        end
      end
      SEEN6: begin
        if ((currentStuffBit != 1) | endPkt) begin
          nextState = SEEN1;
          startEncode = 1;
        end
        else if (currentStuffBit == 1) begin
          nextState = SEND0;
          startEncode = 1;
        end
      end
      SEND0: begin
        nextState = SEEN1;
        startEncode = 1;
        readyForStuff = 0;
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
  (input logic clock, reset_n, twiddleWires, endSeq, currentLevels, send, 
               receive, readDP, readDM,
  output logic donePkt, startDecode, doneReceive, dpWire, dmWire, ldDpDm0, 
               ldDpDm1);

  enum logic [4:0] {SETWIRES, SETX1, SETX2, SETJ, START, CHECKWIRES, SAWX1, 
                    SAWX2, SAWJ} currentState, nextState;

  // Next State and Output Logic
  always_comb begin
    donePkt = 0; dpWire = 1'bz; dmWire = 1'bz;
    unique case (currentState)
      START: begin
        if (send)
          nextState = SETWIRES;
        else if (receive)
          nextState = CHECKWIRES;
        else
          nextState = START;
      end
      SETWIRES: begin
        if (endSeq)
          nextState = SETX1;
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
        nextState = SETX2;
        dpWire = 0;
        dmWire = 0;
      end
      SETX2: begin
        nextState = SETJ;
        dpWire = 0;
        dmWire = 0;
      end
      SETJ: begin
        nextState = SETWIRES;
        dpWire = 1;
        dmWire = 0;
        donePkt = 1;
      end 
      CHECKWIRES: begin
        if (readDP == 1 && readDM == 0) begin
          nextState = CHECKWIRES;
          startDecode = 1;
          ldDpDm1 = 1;
        end
        else if (readDP == 0 && readDM == 1) begin
          nextState = CHECKWIRES;
          startDecode = 1;
          ldDpDm0 = 1;
        end
        else if (readDP == 0 && readDM == 0)
          nextState = SAWX1;
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
        nextState = CHECKWIRES;
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
  (input logic clock, reset_n, startSkip, currentUnstuffBit, doneSkip, 
               doneReceive, 
   output logic takeBit);

  enum logic [2:0] {START, SEEN1, SEEN2, SEEN3, SEEN4, SEEN5, SEEN6, HOLD} 
                    currentState, nextState;

  // Next State and Output Logic
  always_comb begin
    takeBit = 1;
    unique case (currentState)
      START: begin
        if (startSkip) begin
          nextState = SEEN1;
          takeBit = 0;
        end
        else if (~startSkip) begin
          nextState = START;
          takeBit = 0;
        end
      end
      SEEN1: begin
        if (doneReceive) begin
          nextState = START;
        end
        else if (~doneSkip || (doneSkip && (currentUnstuffBit != 1))) begin
          nextState = SEEN1;
          takeBit = 1;
        end
        else if (doneSkip && (currentUnstuffBit == 1)) begin
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
        end
        else begin
          nextState = START;
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
               storeEndp, receiveBit);

  enum logic [3:0] {START, GETSYNC, GETPID, GETEOP, GETPAY, GETADDR, 
                    GETCRC16, GETENDP, GETCRC5} 
                    currentState, nextState;

  // Next State and Output Logic
  always_comb begin
    ldReceiveSync = 0; ldReceivePid = 0; ldReceivePay = 0; ldReceiveAddr = 0; 
    ldReceiveEndp = 0; ldReceiveCRC5 = 0; ldReceiveCRC16 = 0; receiveCRC5 = 0; 
    receiveCRC16 = 0; storeSync = 0; storePid = 0; storePay = 0; storeAddr = 0; 
    storeEndp = 0; receiveBit = 0;
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
        else if (takeBit && (lengthField != 4)) begin
          nextState = GETPID;
          storePid = 1;
          receiveBit = 1;
        end
        else if (takeBit && (lengthField == 1) && 
                ((finishedPkt.pid == PID_ACK) || (finishedPkt.pid == PID_NAK)))
        begin
          nextState = GETEOP;
          //storePid = 1;
        end
        else if (takeBit && (lengthField == 1) && 
                (finishedPkt.pid == PID_DATA0)) begin
          nextState = GETPAY;
          ldReceivePay = 1;
          //storePid = 1;
        end
        else if (takeBit && (lengthField == 1) && 
                ((finishedPkt.pid == PID_OUT) || (finishedPkt.pid == PID_IN)))
        begin
          nextState = GETADDR;
          ldReceiveAddr = 1;
          //storePid = 1;
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
        else if (takeBit && (lengthField != 1)) begin
          nextState = GETCRC16;
          receiveCRC16 = 1;
        end
        else if (takeBit && (lengthField == 1)) begin
          nextState = GETEOP;
          receiveCRC16 = 1;
        end
      end
      GETENDP: begin
        if (~takeBit)
          nextState = GETENDP;
        else if (takeBit && (lengthField != 1)) begin
          nextState = GETENDP;
          storeEndp = 1;
          receiveCRC5 = 1;
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
        end
        else if (takeBit && (lengthField == 1)) begin
          nextState = GETEOP;
          receiveCRC5 = 1;
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


