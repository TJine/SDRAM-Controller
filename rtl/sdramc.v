module sdramc #(
        parameter DATA_WD = 16,
        parameter ADDR_WD = 13,
        parameter COL_WD  = 10
    ) (
        input                       clk,
        input                       rst_n,
        input                       cmd_valid,
        input  [3 : 0]              cmd,
        input  [11 : 0]             r_cmd_len,
        input  [11 : 0]             w_cmd_len,
        input  [12 : 0]             row_addr,
        input  [9 : 0]              col_addr,
        input  [1 : 0]              cmd_ba,
        output reg                  cmd_ready,

        output [DATA_WD - 1 : 0]    r_data,
        output                      r_last,

        output                      Cke,
        output reg                  Cs_n,
        output reg                  Ras_n,
        output reg                  Cas_n,
        output reg                  We_n,
        output [1 : 0]              Dqm,
        output reg [12 : 0]         Addr,
        output reg [1 : 0]          Ba,
        inout  [DATA_WD - 1 : 0]    Dq
    );
    // Fsm state
    localparam [3 : 0] NOP           = 'd0,
                       IDLE          = 'd1,
                       PRECHARGE     = 'd2,
                       AUTO_REFRESH  = 'd3,
                       LOAD_MODE_REG = 'd4,
                       ACTIVE        = 'd5,
                       READ          = 'd6,
                       WRITE         = 'd7,
                       TERMINATE     = 'd8;

    // Addr[2 : 0] ctrl
    localparam [2 : 0] Burst_length_f = 'b111,
                       Burst_length_8 = 'b011,
                       Burst_length_4 = 'b010,
                       Burst_length_2 = 'b001,
                       Burst_length_1 = 'b000;

    // initial and load mode reg, power_cnt ctrl
    localparam PRECHARGE_1    = 'd10006;
    localparam AUTO_REFRESH_1 = 'd10008;
    localparam AUTO_REFRESH_2 = 'd10015;
    localparam AUTO_REFRESH_3 = 'd782;
    localparam LOAD_MODE_REG_1= 'd10022;
    
    reg [3 : 0]             cstate;
    reg [3 : 0]             nstate;
    reg [14 : 0]            power_cnt; // initial wait time
    reg [11 : 0]            cmd_cnt;     // state latency cnt
    reg [DATA_WD - 1 : 0]   wdata;
    reg                     wlast;
    reg                     wfire;    // write enable
    reg                     ina;
    reg [2 : 0]             Mode_reg;

    wire cmd_fire = cmd_valid && cmd_ready;

    // inout
    assign Dq    = wfire ? wdata : 'bz;
    assign r_data = wfire ? 'bz : Dq;

    assign Cke = 1;
    assign Dqm = 0;

    // pre cmd
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ina      <= 1'b0;
        end
        else if (nstate == LOAD_MODE_REG) begin
            ina      <= 1'b1;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            Mode_reg <= 'd0;
        end
        else if (cstate == LOAD_MODE_REG) begin
            Mode_reg <= Addr[2 : 0];
        end
    end

    // write ctrl and data
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wdata <= 'd0;
        end
        else if (wfire) begin
            wdata <= wdata + 1'b1;
            if (wlast) begin
                wdata <= 'd0;
            end
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wfire <= 1'b0;
        end
        else if (nstate == WRITE) begin
            wfire <= 1'b1;
        end
        else if (wlast) begin
            wfire <= 1'b0;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wlast <= 1'b0;
        end
        else if (cmd_cnt == 1) begin
            wlast <= 1'b1;
        end
        else begin
            wlast <= 1'b0;
        end
    end

    // initial wait cnt
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            power_cnt <= 'd0;
        end
        else if (ina) begin
            power_cnt <= 'd0;
        end
        else begin
            power_cnt <= power_cnt + 1'b1;
        end
    end

    // state latency cnt
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cmd_cnt <= 'd0;
        end
        else begin
            if (cmd_cnt != 0) begin
                cmd_cnt <= cmd_cnt - 1'b1;
            end

            case (nstate)
            ACTIVE : cmd_cnt <= 'd2;

            WRITE  : begin
                if (Mode_reg == Burst_length_f) begin
                    cmd_cnt <= w_cmd_len;
                end
                else begin
                    cmd_cnt <= (1 << Mode_reg) - 1'b1;
                end
            end

            READ : begin
                if (Mode_reg == Burst_length_f) begin
                    cmd_cnt <= r_cmd_len;
                end
                else begin
                    cmd_cnt <= (1 << Mode_reg) - 1'b1;
                end
            end

            PRECHARGE : cmd_cnt <= 'd2;

            LOAD_MODE_REG : cmd_cnt <= 'd1;

            endcase
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cmd_ready <= 1'b0;
        end
        else if (ina) begin
            if (cmd_cnt == 'd0) begin
                cmd_ready <= 1'b1;
            end
            else begin
                cmd_ready <= 1'b0;
            end

            if (cmd_fire) begin
                cmd_ready <= 1'b0;
            end
        end
    end

    // Auto_refrsh
    reg [9 : 0] auto_refrsh_cnt;
    reg         auto_refrsh_valid;
    reg         auto_refrsh_ready;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            auto_refrsh_cnt <= 'd0;
            auto_refrsh_valid <= 1'b0;
        end
        else if (auto_refrsh_cnt == AUTO_REFRESH_3) begin
            auto_refrsh_cnt <= 'd0;
            auto_refrsh_valid <= 1'b1;
        end
        else begin
            auto_refrsh_cnt <= auto_refrsh_cnt + 1;
            auto_refrsh_valid <= 1'b0;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            auto_refrsh_ready <= 1'b0;
        end
        else if (nstate == PRECHARGE) begin
            auto_refrsh_ready <= 1'b1;
        end
        else if ((nstate != NOP) && (nstate != AUTO_REFRESH)) begin
            auto_refrsh_ready <= 1'b0;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cstate <= IDLE;
        end
        else begin
            cstate <= nstate;
        end
    end

    always @(*) begin
        case(cstate)
        IDLE : begin
            if (power_cnt == 'd10005) begin
                nstate = NOP;
            end
        end

        NOP : begin
            case(power_cnt)
            PRECHARGE_1 : nstate = PRECHARGE;

            AUTO_REFRESH_1 : nstate = AUTO_REFRESH;

            AUTO_REFRESH_2 : nstate = AUTO_REFRESH;

            LOAD_MODE_REG_1 : nstate = LOAD_MODE_REG;

            default : nstate = NOP;
            endcase

            if (auto_refrsh_valid && auto_refrsh_ready) begin
                nstate = AUTO_REFRESH;
            end

            if (cmd_fire) begin
                nstate = cmd;
            end
        end

        PRECHARGE     : nstate = NOP;
                
        AUTO_REFRESH  : nstate = NOP;
        
        LOAD_MODE_REG : nstate = NOP;

        READ          : nstate = NOP;

        WRITE         : nstate = NOP;

        ACTIVE        : nstate = NOP;

        TERMINATE     : nstate = NOP;

        default       : nstate = NOP;
        endcase
    end

    always @(*) begin
        case(cstate)
        IDLE : begin
            Cs_n = 1;
        end

        NOP : begin
            Cs_n  = 0;
            Ras_n = 1;
            Cas_n = 1;
            We_n  = 1;
        end

        ACTIVE : begin
            Cs_n  = 0;
            Ras_n = 0;
            Cas_n = 1;
            We_n  = 1;
        end

        READ : begin
            Cs_n  = 0;
            Ras_n = 1;
            Cas_n = 0;
            We_n  = 1;
        end

        WRITE : begin
            Cs_n  = 0;
            Ras_n = 1;
            Cas_n = 0;
            We_n  = 0;
        end

        PRECHARGE : begin
            Cs_n  = 0;
            Ras_n = 0;
            Cas_n = 1;
            We_n  = 0;
        end

        AUTO_REFRESH : begin
            Cs_n  = 0;
            Ras_n = 0;
            Cas_n = 0;
            We_n  = 1;
        end

        LOAD_MODE_REG : begin
            Cs_n  = 0;
            Ras_n = 0;
            Cas_n = 0;
            We_n  = 0;
        end

        TERMINATE : begin
            Cs_n  = 0;
            Ras_n = 1;
            Cas_n = 1;
            We_n  = 1;
        end
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            Addr <= 'd0;
            Ba   <= 'd0;
        end
        else begin
            case(nstate)
            ACTIVE : begin
                Addr <= row_addr;
                Ba   <= cmd_ba;
            end

            READ : begin
                Addr <= {row_addr[10], col_addr};
                Ba   <= cmd_ba;
            end

            WRITE : begin
                Addr <= {row_addr[10], col_addr};
                Ba   <= cmd_ba;
            end

            PRECHARGE : begin
                Addr[10] <=  row_addr[10];
                Ba       <= cmd_ba;
            end

            LOAD_MODE_REG : begin
                Addr <= row_addr;
            end

            default : begin
                Addr <= 'd0;
                Ba   <= 'd0;
            end
            endcase
        end
    end

    mt48lc32m16a2 #(.addr_bits(ADDR_WD), .data_bits(DATA_WD), .col_bits(COL_WD))
    mt48lc32m16a2 (
        .Addr   (Addr),
        .Ba     (Ba),

        .Clk    (clk),

        .Cke    (Cke),
        .Cs_n   (Cs_n),
        .Ras_n  (Ras_n),
        .Cas_n  (Cas_n),
        .We_n   (We_n),

        .Dq     (Dq),
        .Dqm    (Dqm)
    );
endmodule
