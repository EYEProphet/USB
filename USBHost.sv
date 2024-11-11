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

  pkt_t currentPkt;
  logic ldSync, ldPid, ldPay, ldAddr, ldEndp, bitEncodeReg, bitStuffReg,
        nrziReg, dpdmReg, currentBit, ldBitEncode, pktSent, readyForStuff, 
        stopPkt, readyForPkt, ldCRC5, ldCRC16, startCRC5, startCRC16, startPass,
        endPkt, shift, lsb, finishPkt, startEncode, donePkt, twiddleWires, 
        endSeq, ldZero, ldOne;
  logic [63:0] currentField;
  logic [6:0] lengthField;
  logic [4:0] finishedCRC5;
  logic [15:0] finishedCRC16;

  // Calls the OUT packet to be sent to ADDR 63 and ENDP 4
  task prelabRequest();
    pktSent = 1;
    currentPkt.pid = PID_OUT;
    currentPkt.addr = 7'h3F;
    currentPkt.endp = 4'h4;
    wait(stopPkt);
  endtask : prelabRequest

  /* Register to choose with field of the packet we are currently looking at and
  the length of that field */
  always_ff @(posedge clock) begin
    if (~reset_n) begin
      currentField <= '0;
      lengthField <= '0;
    end
    else if (ldSync) begin
      currentField <= 8'b00000001;
      lengthField <= 8;
    end
    else if (ldPid) begin
      currentField <= {~currentPkt.pid, currentPkt.pid};
      lengthField <= 8;
    end
    else if (ldPay) begin
      currentField <= currentPkt.payload;
      lengthField <= 64;
    end
    else if (ldAddr) begin
      currentField <= currentPkt.addr;
      lengthField <= 7;
    end
    else if (ldEndp) begin
      currentField <= currentPkt.endp;
      lengthField <= 4;
    end
    else if (ldCRC5)
      lengthField <= 5;
    else if (ldCRC16)
      lengthField <= 16;
    else if (lsb && shift && lengthField != 0) begin
      currentField <= currentField >> 1;
      lengthField <= lengthField - 1;
    end
    else if (~lsb && shift && lengthField != 0) begin
      currentField <= currentField << 1;
      lengthField <= lengthField - 1;
    end
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

  // NRZI Register
  always_ff @(posedge clock) begin
    if (~reset_n)
      nrziReg <= 0;
    else if (ldZero)
      nrziReg <= 0;
    else if (ldOne)
      nrziReg <= 1;
  end

  /* Takes the packet field and sends a bit stream to the CRC, Bit Stuffer, NZRI
  and the DP/DM */
  bitStreamEncoding encode(.clock, .reset_n, .pktSent, .readyForStuff, .stopPkt, 
                          .lengthField, .currentField, .finishedCRC5, 
                          .finishedCRC16, .currentPkt, .readyForPkt, .ldSync, 
                          .ldPid, .ldPay, .ldAddr, .ldEndp, .ldCRC5, .ldCRC16, 
                          .startCRC5, .startCRC16, .startPass, .endPkt, .shift, 
                          .lsb, .currentBit);

  // Performs the CRC5 on non DATA0 packets
  CRC5 errorDetect5(.clock, .reset_n, .enable(startCRC5), .currentBit, 
                    .send(shift), .finishedCRC5);

  // Performs the CRC16 on the DATA0 packets
  CRC16 errorDetect16(.clock, .reset_n, .enable(startCRC16), .currentBit, 
                      .send(shift), .finishedCRC16);

  // Takes the bit stream given and inserts a 0 for every 6 conesecutive 1s
  bitStuffing stuff(.clock, .reset_n, .startPass, .currentStuffBit(bitStuffReg), 
                    .endPkt, .finishPkt, .readyForStuff, .startEncode, 
                    .stopPkt);

  // Takes the bit stream and performs NRZI encoding on it
  nrziEncoding nrzi(.clock, .reset_n, .startEncode, .donePkt, 
                    .currentEncodeBit(bitStuffReg), .twiddleWires, .endSeq, 
                    .finishPkt, .ldZero, .ldOne);

  // Sets D+ and D- wires to their correct values based on the bit stream
  dpDMFSM setWires(.clock, .reset_n, .twiddleWires, .endSeq,
                   .currentLevels(nrziReg), .donePkt, .dpWire(wires.DP), 
                   .dmWire(wires.DM));

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
  (input logic clock, reset_n, enable, currentBit, send,
  output logic [4:0] finishedCRC5);

  logic [2:0] firstThreeReg;
  logic [1:0] lastTwoReg;
  logic lastTwoXor, firstThreeXor;

  assign lastTwoXor = (firstThreeReg[2] ^ currentBit);
  assign firstThreeXor = (lastTwoReg[1] ^ lastTwoXor);

  /* Creates a register to hold the final residue value once the CRC5 is done
  calculating */
  always_ff @(posedge clock) begin
    if (~reset_n)
      finishedCRC5 <= '1;
    else if (enable)
      finishedCRC5 <= ~({((firstThreeReg << 1) | firstThreeXor), 
                       ((lastTwoReg << 1) | lastTwoXor)});
    else if (send)
      finishedCRC5 <= finishedCRC5 << 1;
  end

  // Creates the last two flip flops in the CRC5 hardware
  always_ff @(posedge clock) begin
    if (~reset_n)
      lastTwoReg <= '1;
    else if (enable)
      lastTwoReg <= ((lastTwoReg << 1) | lastTwoXor);
  end

  // Creates the first three flip flops in the CRC5 hardware
  always_ff @(posedge clock) begin
    if (~reset_n)
      firstThreeReg <= '1;
    else if (enable)
      firstThreeReg <= ((firstThreeReg << 1) | firstThreeXor);
  end

endmodule: CRC5

/* Mimics the CRC16 hardware in order to calculate the residue for the DATA0 
packets */
module CRC16
  (input logic clock, reset_n, enable, currentBit, send,
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
    else if (enable)
      finishedCRC16 <= ~({((firstOneReg << 1) | firstOneXor), 
                         ((middleThirteenReg << 1) | middleThirteenXor), 
                         ((lastTwoReg << 1) | lastTwoXor)});
    else if (send)
      finishedCRC16 <= finishedCRC16 << 1;
  end

  // Creates the last two flip flops in the CRC16 hardware
  always_ff @(posedge clock) begin
    if (~reset_n)
      lastTwoReg <= '1;
    else if (enable)
      lastTwoReg <= ((lastTwoReg << 1) | lastTwoXor);
  end

  // Creates the middle thirteen flip flops in the CRC16 hardware
  always_ff @(posedge clock) begin
    if (~reset_n)
      middleThirteenReg <= '1;
    else if (enable)
      middleThirteenReg <= ((middleThirteenReg << 1) | middleThirteenXor);
  end

  // Creates the first flip flop in the CRC16 hardware
  always_ff @(posedge clock) begin
    if (~reset_n)
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
  input logic [63:0] currentField,
  input logic [4:0] finishedCRC5,
  input logic [15:0] finishedCRC16,
  input pkt_t currentPkt,
  output logic readyForPkt, ldSync, ldPid, ldPay, ldAddr, ldEndp, 
               ldCRC5, ldCRC16, startCRC5, startCRC16, startPass, endPkt, 
               shift, lsb, currentBit);

  enum logic [3:0] {START, SENDSYNC, SENDPID, SENDEOP, SENDPAY, SENDADDR, 
                    SENDCRC16, SENDENDP, SENDCRC5} currentState, nextState;

  // Next State and Output Logic
  always_comb begin
    readyForPkt = 0; ldSync = 0; ldPid = 0; ldPay = 0; ldAddr = 0; 
    ldEndp = 0; ldCRC5 = 0; ldCRC16 = 0; startCRC5 = 0; startCRC16 = 0; 
    startPass = 0; endPkt = 0; shift = 0; lsb = 0;
    unique case (currentState)
      START: begin 
        if (pktSent) begin
          nextState = SENDSYNC;
          ldSync = 1;
          readyForPkt = 0;
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
          lsb = 0;
          shift = 1; 
          currentBit = currentField[7];
        end
        else if (readyForStuff && (lengthField == 1)) begin
          nextState = SENDPID;
          ldPid = 1;
          startPass = 1;
          lsb = 0;
          currentBit = currentField[7];
        end
      end
      SENDPID: begin
        if (~readyForStuff)
          nextState = SENDPID;
        else if (readyForStuff && (lengthField != 1)) begin
          nextState = SENDPID;
          startPass = 1;
          lsb = 1;
          shift = 1; 
          currentBit = currentField[0];
        end
        else if (readyForStuff && (lengthField == 1) && 
                ((currentPkt.pid == PID_ACK) || (currentPkt.pid == PID_NAK)))
        begin
          nextState = SENDEOP;
          startPass = 1;
          lsb = 1; 
          currentBit = currentField[0];
        end
        else if (readyForStuff && (lengthField == 1) && 
                (currentPkt.pid == PID_DATA0)) begin
          nextState = SENDPAY;
          startPass = 1;
          ldPay = 1;
          lsb = 1; 
          currentBit = currentField[0];
        end
        else if (readyForStuff && (lengthField == 1) && 
                ((currentPkt.pid == PID_OUT) || (currentPkt.pid == PID_IN)))
        begin
          nextState = SENDADDR;
          startPass = 1;
          ldAddr = 1;
          lsb = 1; 
          currentBit = currentField[0];
        end
      end
      SENDEOP: begin
        if (~stopPkt) begin
          nextState = SENDEOP;
          endPkt = 1;
        end
        else if (stopPkt)
          nextState = START;
      end
      SENDPAY: begin
        if (~readyForStuff)
          nextState = SENDPAY;
        else if (readyForStuff && (lengthField != 1)) begin
          nextState = SENDPAY;
          lsb = 1;
          shift = 1; 
          startCRC16 = 1;
          currentBit = currentField[0];
        end
        else if (readyForStuff && (lengthField == 1)) begin
          nextState = SENDCRC16;
          lsb = 1; 
          startCRC16 = 1;
          ldCRC16 = 1;
          currentBit = currentField[0];
        end
      end
      SENDADDR: begin
        if (~readyForStuff) begin
          nextState = SENDADDR;
        end
        else if (readyForStuff && (lengthField != 1)) begin
          nextState = SENDADDR;
          lsb = 1;
          shift = 1; 
          startCRC5 = 1;
          currentBit = currentField[0];
        end
        else if (readyForStuff && (lengthField == 1)) begin
          nextState = SENDENDP;
          ldEndp = 1;
          lsb = 1; 
          startCRC5 = 1;
          currentBit = currentField[0];
        end
      end
      SENDCRC16: begin
        if (~readyForStuff)
          nextState = SENDCRC16;
        else if (readyForStuff && (lengthField != 1)) begin
          nextState = SENDCRC16;
          shift = 1;
          currentBit = finishedCRC16[15];
        end
        else if (readyForStuff && (lengthField == 1)) begin
          nextState = SENDEOP;
          shift = 1; 
          currentBit = finishedCRC16[15];
        end
      end
      SENDENDP: begin
        if (~readyForStuff)
          nextState = SENDENDP;
        else if (readyForStuff && (lengthField != 1)) begin
          nextState = SENDENDP;
          lsb = 1;
          shift = 1; 
          startCRC5 = 1;
          currentBit = currentField[0];
        end
        else if (readyForStuff && (lengthField == 1)) begin
          nextState = SENDCRC5;
          lsb = 1;
          startCRC5 = 1;
          ldCRC5 = 1;
          currentBit = currentField[0];
        end
      end
      SENDCRC5: begin
        if (~readyForStuff)
          nextState = SENDCRC5;
        else if (readyForStuff && (lengthField != 1)) begin
          nextState = SENDCRC5;
          shift = 1; 
          currentBit = finishedCRC5[4];
        end
        else if (readyForStuff && (lengthField == 1)) begin
          nextState = SENDEOP;
          shift = 1; 
          currentBit = finishedCRC5[4];
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
   output logic readyForStuff, startEncode, stopPkt);

  enum logic [2:0] {START, PASS, SEEN1, SEEN2, SEEN3, SEEN4, SEEN5, SEEN6} 
                    currentState, nextState;

  // Next State and Output Logic
  always_comb begin
    readyForStuff = 1; startEncode = 0; stopPkt = 0;
    unique case (currentState)
      START: begin
        if (startPass) begin
          nextState = PASS;
          startEncode = 1;
        end
        else if (~startPass) begin
          nextState = START;
        end
      end
      PASS: begin
        if (finishPkt) begin
          nextState = START;
          stopPkt = 1;
        end
        else if (endPkt) begin
          nextState = PASS;
        end
        else if (startPass) begin
          nextState = PASS;
          startEncode = 1;
        end
        else if (~startPass && (currentStuffBit != 1)) begin
          nextState = PASS;
          startEncode = 1;
        end
        else if (~startPass && (currentStuffBit == 1)) begin
          nextState = SEEN1;
          startEncode = 1;
        end
      end
      SEEN1: begin
        if (endPkt) begin
          nextState = PASS;
          startEncode = 1;
        end
        else if (currentStuffBit != 1) begin
          nextState = SEEN1;
          startEncode = 1;
        end
        else if (currentStuffBit == 1) begin
          nextState = SEEN2;
          startEncode = 1;
        end
      end
      SEEN2: begin
        if (endPkt) begin
          nextState = PASS;
          startEncode = 1;
        end
        else if (currentStuffBit != 1) begin
          nextState = SEEN2;
          startEncode = 1;
        end
        else if (currentStuffBit == 1) begin
          nextState = SEEN3;
          startEncode = 1;
        end
      end
      SEEN3: begin
        if (endPkt) begin
          nextState = PASS;
          startEncode = 1;
        end
        else if (currentStuffBit != 1) begin
          nextState = SEEN3;
          startEncode = 1;
        end
        else if (currentStuffBit == 1) begin
          nextState = SEEN4;
          startEncode = 1;
        end
      end
      SEEN4: begin
        if (endPkt) begin
          nextState = PASS;
          startEncode = 1;
        end
        else if (currentStuffBit != 1) begin
          nextState = SEEN4;
          startEncode = 1;
        end
        else if (currentStuffBit == 1) begin
          nextState = SEEN5;
          startEncode = 1;
        end
      end
      SEEN5: begin
        if (endPkt) begin
          nextState = PASS;
          startEncode = 1;
        end
        else if (currentStuffBit != 1) begin
          nextState = SEEN5;
          startEncode = 1;
        end
        else if (currentStuffBit == 1) begin
          nextState = SEEN6;
          startEncode = 1;
        end
      end
      SEEN6: begin
        if (currentStuffBit == 1) begin
          nextState = PASS;
          startEncode = 1;
          readyForStuff = 0;
        end
        else if (endPkt) begin
          nextState = PASS;
          startEncode = 1;
        end
        else if (currentStuffBit != 1) begin
          nextState = SEEN6;
          startEncode = 1;
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
  (input logic clock, reset_n, twiddleWires, endSeq, currentLevels,
  output logic donePkt, dpWire, dmWire);

  enum logic [1:0] {SETWIRES, SETX1, SETX2, SETJ} currentState, nextState;

  // Next State and Output Logic
  always_comb begin
    donePkt = 0; dpWire = 1'bz; dmWire = 1'bz;
    unique case (currentState)
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
    endcase
  end

  // State Register
  always_ff @(posedge clock) begin
    if (~reset_n)
      currentState <= SETWIRES;
    else
      currentState <= nextState;
  end
endmodule: dpDMFSM