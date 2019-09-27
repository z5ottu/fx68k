localparam CF = 0, VF = 1, ZF = 2, NF = 3, XF = 4, SF = 13;

localparam UADDR_WIDTH = 10;
localparam UROM_WIDTH = 17;
localparam UROM_DEPTH = 1024;

localparam NADDR_WIDTH = 9;
localparam NANO_WIDTH = 68;
localparam NANO_DEPTH = 336;

localparam BSER1_NMA = 'h003;
localparam RSTP0_NMA = 'h002;
localparam HALT1_NMA = 'h001;
localparam TRAC1_NMA = 'h1C0;
localparam ITLX1_NMA = 'h1C4;

localparam TVN_SPURIOUS = 12;
localparam TVN_AUTOVEC = 13;
localparam TVN_INTERRUPT = 15;

localparam NANO_DOB_DBD = 2'b01;
localparam NANO_DOB_ADB = 2'b10;
localparam NANO_DOB_ALU = 2'b11;

//
// Data bus I/O
// At a separate module because it is a bit complicated and the timing is special.
// Here we do the low/high byte mux and the special case of MOVEP.
//
// Original implementation is rather complex because both the internal and external buses are bidirectional.
// Input is latched async at the EDB register.
// We capture directly from the external data bus to the internal registers (IRC & DBIN) on PHI2, starting the external S7 phase, at a T4 internal period.

module dataIo(
    input Clks_clk,
    input Clks_extReset,         // External sync reset on emulated system
    input Clks_pwrUp,            // Asserted together with reset on emulated system coldstart
    input Clks_enPhi1,
    input Clks_enPhi2,           // Clock enables. Next cycle is PHI1 or PHI2
    input enT1, enT2, enT3, enT4,

    input Nanod_permStart,
    input Nanod_waitBusFinish,
    input Nanod_isWrite,
    input Nanod_busByte,
    input Nanod_isRmc,
    input Nanod_noLowByte, Nanod_noHighByte,
    input Nanod_updTpend, Nanod_clrTpend,
    input Nanod_tvn2Ftu, Nanod_const2Ftu,
    input Nanod_ftu2Dbl, Nanod_ftu2Abl,
    input Nanod_abl2Pren, Nanod_updPren,
    input Nanod_inl2psw, Nanod_ftu2Sr, Nanod_sr2Ftu, Nanod_ftu2Ccr, Nanod_pswIToFtu,
    input Nanod_ird2Ftu, Nanod_ssw2Ftu,
    input Nanod_initST,
    input Nanod_Ir2Ird,
    input Nanod_auClkEn, Nanod_noSpAlign,
    input [2:0] Nanod_auCntrl,
    input Nanod_todbin, Nanod_toIrc,
    input Nanod_dbl2Atl, Nanod_abl2Atl, Nanod_atl2Abl, Nanod_atl2Dbl,
    input Nanod_abh2Ath, Nanod_dbh2Ath,
    input Nanod_ath2Dbh, Nanod_ath2Abh,
    input Nanod_db2Aob, Nanod_ab2Aob, Nanod_au2Aob,
    input Nanod_aob2Ab, Nanod_updSsw,
    input [1:0] Nanod_dobCtrl,
    input Nanod_abh2reg, Nanod_abl2reg,
    input Nanod_reg2abl, Nanod_reg2abh,
    input Nanod_dbh2reg, Nanod_dbl2reg,
    input Nanod_reg2dbl, Nanod_reg2dbh,
    input Nanod_ssp, Nanod_pchdbh, Nanod_pcldbl, Nanod_pclabl, Nanod_pchabh,
    input Nanod_rxh2dbh, Nanod_rxh2abh,
    input Nanod_dbl2rxl, Nanod_dbh2rxh,
    input Nanod_rxl2db, Nanod_rxl2ab,
    input Nanod_abl2rxl, Nanod_abh2rxh,
    input Nanod_dbh2ryh, Nanod_abh2ryh,
    input Nanod_ryl2db, Nanod_ryl2ab,
    input Nanod_ryh2dbh, Nanod_ryh2abh,
    input Nanod_dbl2ryl, Nanod_abl2ryl,
    input Nanod_rz,
    input Nanod_rxlDbl,
    input [2:0] Nanod_aluColumn,
    input [1:0] Nanod_aluDctrl,
    input Nanod_aluActrl,
    input Nanod_aluInit, Nanod_aluFinish,
    input Nanod_abd2Dcr, Nanod_dcr2Dbd,
    input Nanod_dbd2Alue, Nanod_alue2Dbd,
    input Nanod_dbd2Alub, Nanod_abd2Alub,
    input Nanod_alu2Dbd, Nanod_alu2Abd,
    input Nanod_au2Db, Nanod_au2Ab, Nanod_au2Pc,
    input Nanod_dbin2Abd, Nanod_dbin2Dbd,
    input Nanod_extDbh, Nanod_extAbh,
    input Nanod_ablAbd, Nanod_ablAbh,
    input Nanod_dblDbd, Nanod_dblDbh,
    input Nanod_abdIsByte,


    input        Irdecod_isPcRel,
    input        Irdecod_isTas,
    input        Irdecod_implicitSp,
    input        Irdecod_toCcr,
    input        Irdecod_rxIsDt,
    input        Irdecod_ryIsDt,
    input        Irdecod_rxIsUsp,
    input        Irdecod_rxIsMovem,
    input        Irdecod_movemPreDecr,
    input        Irdecod_isByte,
    input        Irdecod_isMovep,
    input [2:0]  Irdecod_rx,
    input [2:0]  Irdecod_ry,
    input        Irdecod_rxIsAreg,
    input        Irdecod_ryIsAreg,
    input [15:0] Irdecod_ftuConst,
    input [ 5:0] Irdecod_macroTvn,
    input        Irdecod_inhibitCcr,    

    input [15:0] iEdb,
    input aob0,

    input dobIdle,
    input [15:0] dobInput,
    
    output logic [15:0] Irc,    
    output logic [15:0] dbin,
    output logic [15:0] oEdb
    );

    reg [15:0] dob;
    
    // DBIN/IRC
    
    // Timing is different than any other register. We can latch only on the next T4 (bus phase S7).
    // We need to register all control signals correctly because the next ublock will already be started.
    // Can't latch control on T4 because if there are wait states there might be multiple T4 before we latch.
    
    reg xToDbin, xToIrc;
    reg dbinNoLow, dbinNoHigh;
    reg byteMux, isByte_T4;
    
    always_ff @( posedge Clks_clk) begin
    
        // Byte mux control. Can't latch at T1. AOB might be not ready yet.
        // Must latch IRD decode at T1 (or T4). Then combine and latch only at T3.

        // Can't latch at T3, a new IRD might be loaded already at T1.  
        // Ok to latch at T4 if combination latched then at T3
        if( enT4)
            isByte_T4 <= Irdecod_isByte;    // Includes MOVEP from mem, we could OR it here

        if( enT3) begin
            dbinNoHigh <= Nanod_noHighByte;
            dbinNoLow <= Nanod_noLowByte;
            byteMux <= Nanod_busByte & isByte_T4 & ~aob0;
        end
        
        if( enT1) begin
            // If on wait states, we continue latching until next T1
            xToDbin <= 1'b0;
            xToIrc <= 1'b0;         
        end
        else if( enT3) begin
            xToDbin <= Nanod_todbin;
            xToIrc <= Nanod_toIrc;
        end

        // Capture on T4 of the next ucycle
        // If there are wait states, we keep capturing every PHI2 until the next T1
        
        if( xToIrc & Clks_enPhi2)
            Irc <= iEdb;
        if( xToDbin & Clks_enPhi2) begin
            // Original connects both halves of EDB.
            if( ~dbinNoLow)
                dbin[ 7:0] <= byteMux ? iEdb[ 15:8] : iEdb[7:0];
            if( ~dbinNoHigh)
                dbin[ 15:8] <= ~byteMux & dbinNoLow ? iEdb[ 7:0] : iEdb[ 15:8];
        end
    end
    
    // DOB
    logic byteCycle;    
    
    always_ff @( posedge Clks_clk) begin
        // Originaly on T1. Transfer to internal EDB also on T1 (stays enabled upto the next T1). But only on T4 (S3) output enables.
        // It is safe to do on T3, then, but control signals if derived from IRD must be registered.
        // Originally control signals are not registered.
        
        // Wait states don't affect DOB operation that is done at the start of the bus cycle. 

        if( enT4)
            byteCycle <= Nanod_busByte & Irdecod_isByte;        // busIsByte but not MOVEP
        
        // Originally byte low/high interconnect is done at EDB, not at DOB.
        if( enT3 & ~dobIdle) begin
            dob[7:0] <= Nanod_noLowByte ? dobInput[15:8] : dobInput[ 7:0];
            dob[15:8] <= (byteCycle | Nanod_noHighByte) ? dobInput[ 7:0] : dobInput[15:8];
        end
    end
    assign oEdb = dob;

endmodule

// Provides ucode routine entries (A1/A3) for each opcode
// Also checks for illegal opcode and priv violation

// This is one of the slowest part of the processor.
// But no need to optimize or pipeline because the result is not needed until at least 4 cycles.
// IR updated at the least one microinstruction earlier.
// Just need to configure the timing analizer correctly.

module uaddrDecode( 
    input [15:0] opcode,
    output [UADDR_WIDTH-1:0] a1, a2, a3,
    output logic isPriv, isIllegal, isLineA, isLineF,
    output [15:0] lineBmap);
    
    wire [3:0] line = opcode[15:12];
    logic [3:0] eaCol, movEa;

    onehotEncoder4 irLineDecod( line, lineBmap);
    
    assign isLineA = lineBmap[ 'hA];
    assign isLineF = lineBmap[ 'hF];
    
    pla_lined pla_lined( .movEa( movEa), .col( eaCol),
        .opcode( opcode), .lineBmap( lineBmap),
        .palIll( isIllegal), .plaA1( a1), .plaA2( a2), .plaA3( a3) );
    
    // ea decoding   
    assign eaCol = eaDecode( opcode[ 5:0]);
    assign movEa = eaDecode( {opcode[ 8:6], opcode[ 11:9]} );

    // EA decode
    function [3:0] eaDecode;
    input [5:0] eaBits;
    begin
    case( eaBits[ 5:3])
    3'b111:
        case( eaBits[ 2:0])
        3'b000:   eaDecode = 7;            // Absolute short
        3'b001:   eaDecode = 8;            // Absolute long
        3'b010:   eaDecode = 9;            // PC displacement
        3'b011:   eaDecode = 10;           // PC offset
        3'b100:   eaDecode = 11;           // Immediate
        default:  eaDecode = 12;           // Invalid
        endcase
         
    default:   eaDecode = eaBits[5:3];      // Register based EAs
    endcase
    end
    endfunction
    
    
    /*
    Privileged instructions:

    ANDI/EORI/ORI SR
    MOVE to SR
    MOVE to/from USP
    RESET
    RTE
    STOP
    */
    
    always_comb begin
        case( lineBmap)
           
        // ori/andi/eori SR      
        'h01:   isPriv = ((opcode & 16'hf5ff) == 16'h007c);
      
        'h10:
            begin
            // No priority !!!
                if( (opcode & 16'hffc0) == 16'h46c0)        // move to sr
                    isPriv = 1'b1;
                
                else if( (opcode & 16'hfff0) == 16'h4e60)   // move usp
                    isPriv = 1'b1;
        
                else if(    opcode == 16'h4e70 ||           // reset
                            opcode == 16'h4e73 ||           // rte
                            opcode == 16'h4e72)             // stop
                    isPriv = 1'b1;
                else
                    isPriv = 1'b0;
            end
         
        default:   isPriv = 1'b0;
        endcase
    end

    
endmodule


// bin to one-hot, 4 bits to 16-bit bitmap
module onehotEncoder4( input [3:0] bin, output reg [15:0] bitMap);
    always_comb begin
    case( bin)
        'b0000:   bitMap = 16'h0001;
        'b0001:   bitMap = 16'h0002;
        'b0010:   bitMap = 16'h0004;
        'b0011:   bitMap = 16'h0008;
        'b0100:   bitMap = 16'h0010;
        'b0101:   bitMap = 16'h0020;
        'b0110:   bitMap = 16'h0040;
        'b0111:   bitMap = 16'h0080;
        'b1000:   bitMap = 16'h0100;
        'b1001:   bitMap = 16'h0200;
        'b1010:   bitMap = 16'h0400;
        'b1011:   bitMap = 16'h0800;
        'b1100:   bitMap = 16'h1000;
        'b1101:   bitMap = 16'h2000;
        'b1110:   bitMap = 16'h4000;
        'b1111:   bitMap = 16'h8000;
    endcase
    end
endmodule



// priority encoder
// used by MOVEM regmask
// this might benefit from device specific features
// MOVEM doesn't need speed, will read the result 2 CPU cycles after each update.
module pren( mask, hbit);
   parameter size = 16;
   parameter outbits = 4;

   input [size-1:0] mask;
   output reg [outbits-1:0] hbit;
   // output reg idle;
   
   always @( mask) begin
      integer i;
      hbit = 0;
      // idle = 1;
      for( i = size-1; i >= 0; i = i - 1) begin
          if( mask[ i]) begin
             hbit = i;
             // idle = 0;
         end
      end
   end

endmodule



// Microcode sequencer

module sequencer( 
    input Clks_clk,
    input Clks_extReset,         // External sync reset on emulated system
    input Clks_pwrUp,            // Asserted together with reset on emulated system coldstart
    input Clks_enPhi1,
    input Clks_enPhi2,           // Clock enables. Next cycle is PHI1 or PHI2
    input enT3,
    input [UROM_WIDTH-1:0] microLatch,
    input A0Err, BerrA, busAddrErr, Spuria, Avia,
    input Tpend, intPend, isIllegal, isPriv, excRst, isLineA, isLineF,
    input [15:0] psw,
    input prenEmpty, au05z, dcr4, ze, i11,
    input [1:0] alue01,
    input [15:0] Ird,
    input [UADDR_WIDTH-1:0] a1, a2, a3,
    output logic [3:0] tvn,
    output logic [UADDR_WIDTH-1:0] nma);
    
    logic [UADDR_WIDTH-1:0] uNma;
    logic [UADDR_WIDTH-1:0] grp1Nma;    
    logic [1:0] c0c1;
    reg a0Rst;
    wire A0Sel;
    wire inGrp0Exc;
    
    // assign nma = Clks_extReset ? RSTP0_NMA : (A0Err ? BSER1_NMA : uNma);
    // assign nma = A0Err ? (a0Rst ? RSTP0_NMA : BSER1_NMA) : uNma;
    
    // word type I: 16 15 14 13 12 11 10 09 08 07 06 05 04 03 02 01 00
    // NMA :        .. .. 09 08 01 00 05 04 03 02 07 06 .. .. .. .. ..
    
    wire [UADDR_WIDTH-1:0] dbNma = { microLatch[ 14:13], microLatch[ 6:5], microLatch[ 10:7], microLatch[ 12:11]};

    // Group 0 exception.
    // Separated block from regular NMA. Otherwise simulation might depend on order of assigments.
    always_comb begin
        if( A0Err) begin
            if( a0Rst)                  // Reset
                nma = RSTP0_NMA;
            else if( inGrp0Exc)         // Double fault
                nma = HALT1_NMA;
            else                        // Bus or address error
                nma = BSER1_NMA;
        end
        else
            nma = uNma;
    end

    always_comb begin
        // Format II (conditional) or I (direct branch)
        if( microLatch[1])
            uNma = { microLatch[ 14:13], c0c1, microLatch[ 10:7], microLatch[ 12:11]};      
        else
            case( microLatch[ 3:2])
            0:   uNma = dbNma;   // DB
            1:   uNma = A0Sel ? grp1Nma : a1;
            2:   uNma = a2;
            3:   uNma = a3;
            endcase         
    end

    // Format II, conditional, NMA decoding
    wire [1:0] enl = { Ird[6], prenEmpty};      // Updated on T3
    
    wire [1:0] ms0 = { Ird[8], alue01[0]};
    wire [3:0] m01 = { au05z, Ird[8], alue01};
    wire [1:0] nz1 = { psw[ NF], psw[ ZF]};
    wire [1:0] nv  = { psw[ NF], psw[ VF]};
    
    logic ccTest;
    wire [4:0] cbc = microLatch[ 6:2];          // CBC bits
    
    always_comb begin
        case( cbc)
        'h0:    c0c1 = {i11, i11};                      // W/L offset EA, from IRC

        'h1:    c0c1 = (au05z) ? 2'b01 : 2'b11;         // Updated on T3
        'h11:   c0c1 = (au05z) ? 2'b00 : 2'b11;

        'h02:   c0c1 = { 1'b0, ~psw[ CF]};              // C used in DIV
        'h12:   c0c1 = { 1'b1, ~psw[ CF]};

        'h03:   c0c1 = {psw[ ZF], psw[ ZF]};            // Z used in DIVU
        
        'h04:                                           // nz1, used in DIVS
            case( nz1)
               'b00:         c0c1 = 2'b10;
               'b10:         c0c1 = 2'b01;
               'b01,'b11:    c0c1 = 2'b11;
            endcase     

        'h05:   c0c1 = {psw[ NF], 1'b1};                // N used in CHK and DIV
        'h15:   c0c1 = {1'b1, psw[ NF]};
        
        // nz2, used in DIVS (same combination as nz1)
        'h06:   c0c1 = { ~nz1[1] & ~nz1[0], 1'b1};
        
        'h07:                                           // ms0 used in MUL
        case( ms0)
         'b10, 'b00: c0c1 = 2'b11;
         'b01: c0c1 = 2'b01;
         'b11: c0c1 = 2'b10;
        endcase     
        
        'h08:                                           // m01 used in MUL
        case( m01)
        'b0000,'b0001,'b0100,'b0111:  c0c1 = 2'b11;
        'b0010,'b0011,'b0101:         c0c1 = 2'b01;
        'b0110:                       c0c1 = 2'b10;
        default:                      c0c1 = 2'b00;
        endcase

        // Conditional      
        'h09:   c0c1 = (ccTest) ? 2'b11 : 2'b01;
        'h19:   c0c1 = (ccTest) ? 2'b11 : 2'b10;
        
        // DCR bit 4 (high or low word)
        'h0c:   c0c1 = dcr4 ? 2'b01: 2'b11;
        'h1c:   c0c1 = dcr4 ? 2'b10: 2'b11;     
        
        // DBcc done
        'h0a:   c0c1 = ze ? 2'b11 : 2'b00;
        
        // nv, used in CHK
        'h0b:   c0c1 = (nv == 2'b00) ? 2'b00 : 2'b11;
        
        // V, used in trapv
        'h0d:   c0c1 = { ~psw[ VF], ~psw[VF]};      
        
        // enl, combination of pren idle and word/long on IRD
        'h0e,'h1e:
            case( enl)
            2'b00:  c0c1 = 'b10;
            2'b10:  c0c1 = 'b11;
            // 'hx1 result 00/01 depending on condition 0e/1e
            2'b01,2'b11:
                    c0c1 = { 1'b0, microLatch[ 6]};
            endcase
            
        default:                c0c1 = 'X;
        endcase
    end
    
    // CCR conditional
    always_comb begin
        case( Ird[ 11:8])       
        'h0: ccTest = 1'b1;                     // T
        'h1: ccTest = 1'b0;                     // F
        'h2: ccTest = ~psw[ CF] & ~psw[ ZF];    // HI
        'h3: ccTest = psw[ CF] | psw[ZF];       // LS
        'h4: ccTest = ~psw[ CF];                // CC (HS)
        'h5: ccTest = psw[ CF];                 // CS (LO)
        'h6: ccTest = ~psw[ ZF];                // NE
        'h7: ccTest = psw[ ZF];                 // EQ
        'h8: ccTest = ~psw[ VF];                // VC
        'h9: ccTest = psw[ VF];                 // VS
        'ha: ccTest = ~psw[ NF];                // PL
        'hb: ccTest = psw[ NF];                 // MI
        'hc: ccTest = (psw[ NF] & psw[ VF]) | (~psw[ NF] & ~psw[ VF]);              // GE
        'hd: ccTest = (psw[ NF] & ~psw[ VF]) | (~psw[ NF] & psw[ VF]);              // LT
        'he: ccTest = (psw[ NF] & psw[ VF] & ~psw[ ZF]) |
                 (~psw[ NF] & ~psw[ VF] & ~psw[ ZF]);                               // GT
        'hf: ccTest = psw[ ZF] | (psw[ NF] & ~psw[VF]) | (~psw[ NF] & psw[VF]);     // LE
        endcase
    end
    
    // Exception logic
    logic rTrace, rInterrupt;
    logic rIllegal, rPriv, rLineA, rLineF;
    logic rExcRst, rExcAdrErr, rExcBusErr;
    logic rSpurious, rAutovec;
    wire grp1LatchEn, grp0LatchEn;
            
    // Originally control signals latched on T4. Then exception latches updated on T3
    assign grp1LatchEn = microLatch[0] & (microLatch[1] | !microLatch[4]);
    assign grp0LatchEn = microLatch[4] & !microLatch[1];
    
    assign inGrp0Exc = rExcRst | rExcBusErr | rExcAdrErr;
    
    always_ff @( posedge Clks_clk) begin
        if( grp0LatchEn & enT3) begin
            rExcRst <= excRst;
            rExcBusErr <= BerrA;
            rExcAdrErr <= busAddrErr;
            rSpurious <= Spuria;
            rAutovec <= Avia;
        end
        
        // Update group 1 exception latches
        // Inputs from IR decoder updated on T1 as soon as IR loaded
        // Trace pending updated on T3 at the start of the instruction
        // Interrupt pending on T2
        if( grp1LatchEn & enT3) begin
            rTrace <= Tpend;
            rInterrupt <= intPend;
            rIllegal <= isIllegal & ~isLineA & ~isLineF;
            rLineA <= isLineA;
            rLineF <= isLineF;
            rPriv <= isPriv & !psw[ SF];
        end
    end

    // exception priority
    always_comb begin
        grp1Nma = TRAC1_NMA;
        if( rExcRst)
            tvn = '0;                           // Might need to change that to signal in exception
        else if( rExcBusErr | rExcAdrErr)
            tvn = { 1'b1, rExcAdrErr};
            
        // Seudo group 0 exceptions. Just for updating TVN
        else if( rSpurious | rAutovec)
            tvn = rSpurious ? TVN_SPURIOUS : TVN_AUTOVEC;
        
        else if( rTrace)
            tvn = 9;
        else if( rInterrupt) begin
            tvn = TVN_INTERRUPT;
            grp1Nma = ITLX1_NMA;
        end
        else begin
            case( 1'b1)                 // Can't happen more than one of these
            rIllegal:           tvn = 4;
            rPriv:              tvn = 8;
            rLineA:             tvn = 10;
            rLineF:             tvn = 11;
            default:            tvn = 1;        // Signal no group 0/1 exception
            endcase
        end
    end
    
    assign A0Sel = rIllegal | rLineF | rLineA | rPriv | rTrace | rInterrupt;
    
    always_ff @( posedge Clks_clk) begin
        if( Clks_extReset)
            a0Rst <= 1'b1;
        else if( enT3)
            a0Rst <= 1'b0;
    end
    
endmodule



//
// DMA/BUS Arbitration
//

module busArbiter(
        input Clks_clk,
        input Clks_extReset,         // External sync reset on emulated system
        input Clks_pwrUp,            // Asserted together with reset on emulated system coldstart
        input Clks_enPhi1,
        input Clks_enPhi2,           // Clock enables. Next cycle is PHI1 or PHI2

        input BRi, BgackI, Halti, bgBlock,
        output busAvail,
        output logic BGn);
        
    localparam [2:0] DRESET = 3'd0, DIDLE=3'd1, D1=3'd2, D_BR=3'd3, D_BA=3'd4, D_BRA=3'd5, D3=3'd6, D2=3'd7;
    reg [2:0] dmaPhase, next;

    always_comb begin
        case(dmaPhase)
        DRESET: next = DIDLE;
        DIDLE:  begin
                if( bgBlock)
                    next = DIDLE;
                else if( ~BgackI)
                    next = D_BA;
                else if( ~BRi)
                    next = D1;
                else
                    next = DIDLE;
                end

        D_BA:   begin                           // Loop while only BGACK asserted, BG negated here
                if( ~BRi & !bgBlock)
                    next = D3;
                else if( ~BgackI & !bgBlock)
                    next = D_BA;
                else
                    next = DIDLE;
                end
                
        D1:     next = D_BR;                            // Loop while only BR asserted
        D_BR:   next = ~BRi & BgackI ? D_BR : D_BA;     // No direct path to IDLE !
        
        D3:     next = D_BRA;
        D_BRA:  begin                       // Loop while both BR and BGACK asserted
                case( {BgackI, BRi} )
                2'b11:  next = DIDLE;       // Both deasserted
                2'b10:  next = D_BR;        // BR asserted only
                2'b01:  next = D2;          // BGACK asserted only
                2'b00:  next = D_BRA;       // Stay here while both asserted
                endcase
                end
                
        // Might loop here if both deasserted, should normally don't arrive here anyway?
        // D2:      next = (BgackI & BRi) | bgBlock ? D2: D_BA;

        D2:     next = D_BA;
        
        default:    next = DIDLE;           // Should not reach here normally       
        endcase
    end
        
    logic granting;
    always_comb begin
        case( next)
        D1, D3, D_BR, D_BRA:    granting = 1'b1;
        default:                granting = 1'b0;
        endcase
    end
    
    reg rGranted;
    assign busAvail = Halti & BRi & BgackI & ~rGranted;
        
    always_ff @( posedge Clks_clk) begin
        if( Clks_extReset) begin
            dmaPhase <= DRESET;
            rGranted <= 1'b0;
        end
        else if( Clks_enPhi2) begin
            dmaPhase <= next;
            // Internal signal changed on PHI2
            rGranted <= granting;
        end

        // External Output changed on PHI1
        if( Clks_extReset)
            BGn <= 1'b1;
        else if( Clks_enPhi1)
            BGn <= ~rGranted;
        
    end
            
endmodule



module busControl( 
        //input s_clks Clks, 
        input Clks_clk,
        input Clks_extReset,         // External sync reset on emulated system
        input Clks_pwrUp,            // Asserted together with reset on emulated system coldstart
        input Clks_enPhi1,
        input Clks_enPhi2,           // Clock enables. Next cycle is PHI1 or PHI2

        input enT1, 
        input enT4,
        input permStart, permStop, iStop,
        input aob0,
        input isWrite, isByte, isRmc,
        input busAvail,
        output bgBlock,
        output busAddrErr,
        output waitBusCycle,
        output busStarting,         // Asserted during S0
        output logic addrOe,        // Asserted from S1 to the end, whole bus cycle except S0
        output bciWrite,            // Used for SSW on bus/addr error
        
        input rDtack, BeDebounced, Vpai,
        output ASn, output LDSn, output UDSn, eRWn);

    reg rAS, rLDS, rUDS, rRWn;
    assign ASn = rAS;
    assign LDSn = rLDS;
    assign UDSn = rUDS;
    assign eRWn = rRWn;

    reg dataOe;
        
    reg bcPend;
    reg isWriteReg, bciByte, isRmcReg, wendReg;
    assign bciWrite = isWriteReg;
    reg addrOeDelay;
    reg isByteT4;

    wire canStart, busEnd;
    wire bcComplete, bcReset;
    
    wire isRcmReset = bcComplete & bcReset & isRmcReg;
    
    assign busAddrErr = aob0 & ~bciByte;
    
    // Bus retry not really supported.
    // It's BERR and HALT and not address error, and not read-modify cycle.
    wire busRetry = ~busAddrErr & 1'b0;
    localparam [2:0] SRESET =3'd0, SIDLE=3'd1, S0=3'd2, S2=3'd3, S4=3'd4, S6=3'd5, SRMC_RES=3'd6;
    reg [2:0] busPhase, next;

    always_ff @( posedge Clks_clk) begin
        if( Clks_extReset)
            busPhase <= SRESET;
        else if( Clks_enPhi1)
            busPhase <= next;
    end
    
    always_comb begin
        case( busPhase)
        SRESET:     next = SIDLE;
        SRMC_RES:   next = SIDLE;           // Single cycle special state when read phase of RMC reset
        S0:         next = S2;
        S2:         next = S4;
        S4:         next = busEnd ? S6 : S4;
        S6:         next = isRcmReset ? SRMC_RES : (canStart ? S0 : SIDLE);
        SIDLE:      next = canStart ? S0 : SIDLE;
        default:    next = SIDLE;
        endcase
    end 
    
    // Idle phase of RMC bus cycle. Might be better to just add a new state
    wire rmcIdle = (busPhase == SIDLE) & ~ASn & isRmcReg;
        
    assign canStart = (busAvail | rmcIdle) & (bcPend | permStart) & !busRetry & !bcReset;
        
    wire busEnding = (next == SIDLE) | (next == S0);
    
    assign busStarting = (busPhase == S0);
        
    // term signal (DTACK, BERR, VPA, adress error)
    assign busEnd = ~rDtack | iStop;
                
    // bcComplete asserted on raising edge of S6 (together with SNC).
    assign bcComplete = (busPhase == S6);
    
    // Clear bus info latch on completion (regular or aborted) and no bus retry (and not PHI1).
    // bciClear asserted half clock later on PHI2, and bci latches cleared async concurrently
    wire bciClear = bcComplete & ~busRetry;
    
    // Reset on reset or (berr & berrDelay & (not halt or rmc) & not 6800 & in bus cycle) (and not PHI1)
    assign bcReset = Clks_extReset | (addrOeDelay & BeDebounced & Vpai);
        
    // Enable uclock only on S6 (S8 on Bus Error) or not bciPermStop
    assign waitBusCycle = wendReg & !bcComplete;
    
    // Block Bus Grant when starting new bus cycle. But No need if AS already asserted (read phase of RMC)
    // Except that when that RMC phase aborted on bus error, it's asserted one cycle later!
    assign bgBlock = ((busPhase == S0) & ASn) | (busPhase == SRMC_RES);
    
    always_ff @( posedge Clks_clk) begin
        if( Clks_extReset) begin
            addrOe <= 1'b0;
        end
        else if( Clks_enPhi2 & ( busPhase == S0))           // From S1, whole bus cycle except S0
                addrOe <= 1'b1;
        else if( Clks_enPhi1 & (busPhase == SRMC_RES))
            addrOe <= 1'b0;
        else if( Clks_enPhi1 & ~isRmcReg & busEnding)
            addrOe <= 1'b0;
            
        if( Clks_enPhi1)
            addrOeDelay <= addrOe;
        
        if( Clks_extReset) begin
            rAS <= 1'b1;
            rUDS <= 1'b1;
            rLDS <= 1'b1;
            rRWn <= 1'b1;
            dataOe <= '0;
        end
        else begin

            if( Clks_enPhi2 & isWriteReg & (busPhase == S2))
                dataOe <= 1'b1;
            else if( Clks_enPhi1 & (busEnding | (busPhase == SIDLE)) )
                dataOe <= 1'b0;
                        
            if( Clks_enPhi1 & busEnding)
                rRWn <= 1'b1;
            else if( Clks_enPhi1 & isWriteReg) begin
                // Unlike LDS/UDS Asserted even in address error
                if( (busPhase == S0) & isWriteReg)
                    rRWn <= 1'b0;
            end

            // AS. Actually follows addrOe half cycle later!
            if( Clks_enPhi1 & (busPhase == S0))
                rAS <= 1'b0;
            else if( Clks_enPhi2 & (busPhase == SRMC_RES))      // Bus error on read phase of RMC. Deasserted one cycle later
                rAS <= 1'b1;            
            else if( Clks_enPhi2 & bcComplete & ~SRMC_RES)
                if( ~isRmcReg)                                  // Keep AS asserted on the IDLE phase of RMC
                    rAS <= 1'b1;
            
            if( Clks_enPhi1 & (busPhase == S0)) begin
                if( ~isWriteReg & !busAddrErr) begin
                    rUDS <= ~(~bciByte | ~aob0);
                    rLDS <= ~(~bciByte | aob0);
                end
            end
            else if( Clks_enPhi1 & isWriteReg & (busPhase == S2) & !busAddrErr) begin
                    rUDS <= ~(~bciByte | ~aob0);
                    rLDS <= ~(~bciByte | aob0);
            end     
            else if( Clks_enPhi2 & bcComplete) begin
                rUDS <= 1'b1;
                rLDS <= 1'b1;
            end
            
        end
        
    end
    
    // Bus cycle info latch. Needed because uinstr might change if the bus is busy and we must wait.
    // Note that urom advances even on wait states. It waits *after* updating urom and nanorom latches.
    // Even without wait states, ublocks of type ir (init reading) will not wait for bus completion.
    // Originally latched on (permStart AND T1).
    
    // Bus cycle info latch: isRead, isByte, read-modify-cycle, and permStart (bus cycle pending). Some previously latched on T4?
    // permStop also latched, but unconditionally on T1
    
    // Might make more sense to register this outside this module
    always_ff @( posedge Clks_clk) begin
        if( enT4) begin
            isByteT4 <= isByte;
        end
    end
    
    // Bus Cycle Info Latch
    always_ff @( posedge Clks_clk) begin
        if( Clks_pwrUp) begin
            bcPend <= 1'b0;
            wendReg <= 1'b0;    
            isWriteReg <= 1'b0;
            bciByte <= 1'b0;
            isRmcReg <= 1'b0;       
        end
        
        else if( Clks_enPhi2 & (bciClear | bcReset)) begin
            bcPend <= 1'b0;
            wendReg <= 1'b0;
        end
        else begin
            if( enT1 & permStart) begin
                isWriteReg <= isWrite;
                bciByte <= isByteT4;
                isRmcReg <= isRmc & ~isWrite;   // We need special case the end of the read phase only.
                bcPend <= 1'b1;
            end
            if( enT1)
                wendReg <= permStop;
        end
    end
            
endmodule



//
// microrom and nanorom instantiation
//
// There is bit of wasting of resources here. An extra registering pipeline happens that is not needed.
// This is just for the purpose of helping inferring block RAM using pure generic code. Inferring RAM is important for performance.
// Might be more efficient to use vendor specific features such as clock enable.
//

module uRom( input clk, input [UADDR_WIDTH-1:0] microAddr, output logic [UROM_WIDTH-1:0] microOutput);
    reg [UROM_WIDTH-1:0] uRam[ UROM_DEPTH];     
    initial begin
        $readmemb("microrom.mem", uRam);
    end
    
    always_ff @( posedge clk) 
        microOutput <= uRam[ microAddr];
endmodule


module nanoRom( input clk, input [NADDR_WIDTH-1:0] nanoAddr, output logic [NANO_WIDTH-1:0] nanoOutput);
    reg [NANO_WIDTH-1:0] nRam[ NANO_DEPTH];     
    initial begin
        $readmemb("nanorom.mem", nRam);
    end
    
    always_ff @( posedge clk) 
        nanoOutput <= nRam[ nanoAddr];
endmodule


// Translate uaddr to nanoaddr
module microToNanoAddr(
    input [UADDR_WIDTH-1:0] uAddr,
    output [NADDR_WIDTH-1:0] orgAddr);

    wire [UADDR_WIDTH-1:2] baseAddr = uAddr[UADDR_WIDTH-1:2];
    logic [NADDR_WIDTH-1:2] orgBase;
    assign orgAddr = { orgBase, uAddr[1:0]};

    always @( baseAddr)
    begin
        // nano ROM (136 addresses)
        case( baseAddr)

'h00: orgBase = 7'h0 ;
'h01: orgBase = 7'h1 ;
'h02: orgBase = 7'h2 ;
'h03: orgBase = 7'h2 ;
'h08: orgBase = 7'h3 ;
'h09: orgBase = 7'h4 ;
'h0A: orgBase = 7'h5 ;
'h0B: orgBase = 7'h5 ;
'h10: orgBase = 7'h6 ;
'h11: orgBase = 7'h7 ;
'h12: orgBase = 7'h8 ;
'h13: orgBase = 7'h8 ;
'h18: orgBase = 7'h9 ;
'h19: orgBase = 7'hA ;
'h1A: orgBase = 7'hB ;
'h1B: orgBase = 7'hB ;
'h20: orgBase = 7'hC ;
'h21: orgBase = 7'hD ;
'h22: orgBase = 7'hE ;
'h23: orgBase = 7'hD ;
'h28: orgBase = 7'hF ;
'h29: orgBase = 7'h10 ;
'h2A: orgBase = 7'h11 ;
'h2B: orgBase = 7'h10 ;
'h30: orgBase = 7'h12 ;
'h31: orgBase = 7'h13 ;
'h32: orgBase = 7'h14 ;
'h33: orgBase = 7'h14 ;
'h38: orgBase = 7'h15 ;
'h39: orgBase = 7'h16 ;
'h3A: orgBase = 7'h17 ;
'h3B: orgBase = 7'h17 ;
'h40: orgBase = 7'h18 ;
'h41: orgBase = 7'h18 ;
'h42: orgBase = 7'h18 ;
'h43: orgBase = 7'h18 ;
'h44: orgBase = 7'h19 ;
'h45: orgBase = 7'h19 ;
'h46: orgBase = 7'h19 ;
'h47: orgBase = 7'h19 ;
'h48: orgBase = 7'h1A ;
'h49: orgBase = 7'h1A ;
'h4A: orgBase = 7'h1A ;
'h4B: orgBase = 7'h1A ;
'h4C: orgBase = 7'h1B ;
'h4D: orgBase = 7'h1B ;
'h4E: orgBase = 7'h1B ;
'h4F: orgBase = 7'h1B ;
'h54: orgBase = 7'h1C ;
'h55: orgBase = 7'h1D ;
'h56: orgBase = 7'h1E ;
'h57: orgBase = 7'h1F ;
'h5C: orgBase = 7'h20 ;
'h5D: orgBase = 7'h21 ;
'h5E: orgBase = 7'h22 ;
'h5F: orgBase = 7'h23 ;
'h70: orgBase = 7'h24 ;
'h71: orgBase = 7'h24 ;
'h72: orgBase = 7'h24 ;
'h73: orgBase = 7'h24 ;
'h74: orgBase = 7'h24 ;
'h75: orgBase = 7'h24 ;
'h76: orgBase = 7'h24 ;
'h77: orgBase = 7'h24 ;
'h78: orgBase = 7'h25 ;
'h79: orgBase = 7'h25 ;
'h7A: orgBase = 7'h25 ;
'h7B: orgBase = 7'h25 ;
'h7C: orgBase = 7'h25 ;
'h7D: orgBase = 7'h25 ;
'h7E: orgBase = 7'h25 ;
'h7F: orgBase = 7'h25 ;
'h84: orgBase = 7'h26 ;
'h85: orgBase = 7'h27 ;
'h86: orgBase = 7'h28 ;
'h87: orgBase = 7'h29 ;
'h8C: orgBase = 7'h2A ;
'h8D: orgBase = 7'h2B ;
'h8E: orgBase = 7'h2C ;
'h8F: orgBase = 7'h2D ;
'h94: orgBase = 7'h2E ;
'h95: orgBase = 7'h2F ;
'h96: orgBase = 7'h30 ;
'h97: orgBase = 7'h31 ;
'h9C: orgBase = 7'h32 ;
'h9D: orgBase = 7'h33 ;
'h9E: orgBase = 7'h34 ;
'h9F: orgBase = 7'h35 ;
'hA4: orgBase = 7'h36 ;
'hA5: orgBase = 7'h36 ;
'hA6: orgBase = 7'h37 ;
'hA7: orgBase = 7'h37 ;
'hAC: orgBase = 7'h38 ;
'hAD: orgBase = 7'h38 ;
'hAE: orgBase = 7'h39 ;
'hAF: orgBase = 7'h39 ;
'hB4: orgBase = 7'h3A ;
'hB5: orgBase = 7'h3A ;
'hB6: orgBase = 7'h3B ;
'hB7: orgBase = 7'h3B ;
'hBC: orgBase = 7'h3C ;
'hBD: orgBase = 7'h3C ;
'hBE: orgBase = 7'h3D ;
'hBF: orgBase = 7'h3D ;
'hC0: orgBase = 7'h3E ;
'hC1: orgBase = 7'h3F ;
'hC2: orgBase = 7'h40 ;
'hC3: orgBase = 7'h41 ;
'hC8: orgBase = 7'h42 ;
'hC9: orgBase = 7'h43 ;
'hCA: orgBase = 7'h44 ;
'hCB: orgBase = 7'h45 ;
'hD0: orgBase = 7'h46 ;
'hD1: orgBase = 7'h47 ;
'hD2: orgBase = 7'h48 ;
'hD3: orgBase = 7'h49 ;
'hD8: orgBase = 7'h4A ;
'hD9: orgBase = 7'h4B ;
'hDA: orgBase = 7'h4C ;
'hDB: orgBase = 7'h4D ;
'hE0: orgBase = 7'h4E ;
'hE1: orgBase = 7'h4E ;
'hE2: orgBase = 7'h4F ;
'hE3: orgBase = 7'h4F ;
'hE8: orgBase = 7'h50 ;
'hE9: orgBase = 7'h50 ;
'hEA: orgBase = 7'h51 ;
'hEB: orgBase = 7'h51 ;
'hF0: orgBase = 7'h52 ;
'hF1: orgBase = 7'h52 ;
'hF2: orgBase = 7'h52 ;
'hF3: orgBase = 7'h52 ;
'hF8: orgBase = 7'h53 ;
'hF9: orgBase = 7'h53 ;
'hFA: orgBase = 7'h53 ;
'hFB: orgBase = 7'h53 ;

            default:
                orgBase = 'X;
        endcase
    end

endmodule
