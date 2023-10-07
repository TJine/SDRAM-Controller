module sdramc #(
        parameter DATA_WD = 16,
        parameter ADDR_WD = 13,
        parameter COL_WD  = 10
    ) (
        input                                       clk,
        input                                       rst_n,

        // AXI interface
        // Write
        input  wire                                 S_AXI_AWVALID,      // 1
        input  wire  [31 : 0]                       S_AXI_AWADDR,       // 1
        input  wire  [7 : 0]                        S_AXI_AWLEN,        // 1
        output wire                                 S_AXI_AWREADY,      // 1

        input  wire                                 S_AXI_WVALID,       // 1
        input  wire  [(DATA_WD/8)-1 : 0]            S_AXI_WSTRB,        // 1
        input  wire  [DATA_WD - 1 : 0]              S_AXI_WDATA,        // 1
        input  wire                                 S_AXI_WLAST,        // 1
        output wire                                 S_AXI_WREADY,       // 1

        output wire                                 S_AXI_BVALID,       // 1
        output wire  [1 : 0]                        S_AXI_BRESP,        // 1
        input  wire                                 S_AXI_BREADY,       // 1

        // Read
        input  wire                                 S_AXI_ARVALID,
        input  wire  [31 : 0]                       S_AXI_ARADDR,
        input  wire  [7 : 0]                        S_AXI_ARLEN,
        output wire                                 S_AXI_ARREADY,

        output wire                                 S_AXI_RVALID,
        output wire  [1 : 0]                        S_AXI_RRESP,
        output wire  [DATA_WD - 1 : 0]              S_AXI_RDATA,
        output wire                                 S_AXI_RLAST,
        input  wire                                 S_AXI_RREADY,

        // SDRAM interface
        output                                      Cke,
        output reg                                  Cs_n,
        output reg                                  Ras_n,
        output reg                                  Cas_n,
        output reg                                  We_n,

        output     [1 : 0]                          Dqm,
        output reg [ADDR_WD - 1 : 0]                Addr,
        output reg [1 : 0]                          Ba,
        inout      [DATA_WD - 1 : 0]                Dq           
    );

    localparam NOP          = 0;
    localparam IDLE         = 1;
    localparam PRECHARGE    = 2;
    localparam AUTO_REFRESH = 3;
    localparam LOAD_MODE_REG= 4;
    localparam ACTIVE       = 5;
    localparam READ         = 6;
    localparam WRITE        = 7;
    localparam TERMINATE    = 8;

    localparam PRECHARGE_1     = 'd10006;
    localparam AUTO_REFRESH_1  = 'd10008;
    localparam AUTO_REFRESH_2  = 'd10015;
    localparam AUTO_REFRESH_3  = 'd782;
    localparam LOAD_MODE_REG_1 = 'd10022;

    localparam Burst_length_f = 'b111;
    localparam Burst_length_8 = 'b011;
    localparam Burst_length_4 = 'b010;
    localparam Burst_length_2 = 'b001;
    localparam Burst_length_1 = 'b000;

    reg                     cmd;       // 1 
    reg                     cmd_valid; // 1
    reg                     cmd_ready; // 1
    reg  [7 : 0]            r_cmd_len; // 1
    reg  [7 : 0]            w_cmd_len; // 1
    reg  [ADDR_WD - 1 : 0]  row_addr;  // 1
    reg  [COL_WD - 1 : 0]   col_addr;  // 1
    reg  [1 : 0]            cmd_ba;    // 1

    reg                     r_complete;
    reg                     w_complete;
    reg [15 : 0]            cmd_seq;
    reg [3 : 0]             cstate;
    reg [3 : 0]             nstate;
    reg [14 : 0]            power_cnt;
    reg [11 : 0]            t_cnt;
    reg [9 : 0]             ter_cnt;
    reg [DATA_WD - 1 : 0]   w_data;
    reg                     w_last;
    reg                     w_fire;
    reg                     r_fire;
    reg                     ina;
    reg [2 : 0]             Mode_reg;
    reg                     axi_arready;
    reg                     axi_awready;
    reg                     axi_bvalid;

    wire                    cmd_fire = cmd_valid && cmd_ready;
    wire                    axi_rd_fire = S_AXI_ARVALID && S_AXI_ARREADY;
    wire                    axi_wr_fire = S_AXI_AWVALID && S_AXI_AWREADY;
    wire [DATA_WD - 1 : 0]  r_data;

    assign S_AXI_ARREADY = axi_arready;
    assign S_AXI_AWREADY = axi_awready;
    assign S_AXI_WREADY  = w_fire;

    assign S_AXI_RVALID = r_fire;
    assign S_AXI_RRESP  = 2'b00;
    assign S_AXI_BRESP   = 2'b00;
    assign S_AXI_RDATA  = r_data;
    assign S_AXI_RLAST  = r_fire && w_last;
    assign S_AXI_BVALID = axi_bvalid;

    // inout
    assign Dq = w_fire ? w_data : 'bz;
    assign r_data = w_fire ? 'bz : Dq;

    assign Cke = 1;
    assign Dqm = 0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            axi_bvalid <= 0;
        end
        else if (w_fire && w_last) begin
            axi_bvalid <= 1;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r_cmd_len <= 'b0;
            row_addr <= 'b0;
            col_addr <= 'b0;
            cmd_ba <= 'b0;
        end
        else if (axi_rd_fire) begin
            r_cmd_len <= S_AXI_ARLEN;
            row_addr <= S_AXI_ARADDR[22 : 10] + 1;
            col_addr <= S_AXI_ARADDR[9 : 0];
            cmd_ba <= S_AXI_ARADDR[24 : 23];
        end
        else if (axi_wr_fire) begin
            r_cmd_len <= S_AXI_AWLEN;
            row_addr <= S_AXI_AWADDR[22 : 10] + 1;
            col_addr <= S_AXI_AWADDR[9 : 0];
            cmd_ba <= S_AXI_AWADDR[24 : 23];
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cmd <= 0;
        end
        else if (cmd_fire) begin
            cmd <= cmd_seq[23 : 20];
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cmd_seq <= 'b0;
        end
        else if (cmd_fire) begin
            cmd_seq <= {cmd_seq[11 : 0], 4'd0};
        end
        else if (axi_rd_fire) begin
            cmd_seq <= {4'd5, 4'd7, 4'd8, 4'd2};
        end
        else if (axi_wr_fire) begin
            cmd_seq <= {4'd5, 4'd6, 4'd8, 4'd2};
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r_complete <= 0;
        end
        else if (cmd == 4'h0 && cstate == NOP) begin
            if (w_last) begin
                r_complete <= 1;
            end
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            w_complete <= 0;
        end
        else if (cmd == 4'h0 && cstate == NOP) begin
            if (w_last) begin
                w_complete <= 1;
            end
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            axi_arready <= 1;
        end
        else if (axi_rd_fire) begin
            axi_arready <= 0;
        end
        else if (r_complete) begin
            axi_arready <= 1;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            axi_awready <= 1;
        end
        else if (axi_wr_fire) begin
            axi_awready <= 0;
        end
        else if (w_complete) begin
            axi_awready <= 1;
        end
    end

    

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            Mode_reg <= 'b0;
        end
        else if (cstate == LOAD_MODE_REG) begin
            Mode_reg <= Addr[2 : 0];
        end
    end

    // pre_cmd
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ina <= 0;
        end
        else if (nstate == LOAD_MODE_REG) begin
            ina <= 1;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            w_data <= 'b0;
        end
        else if (w_fire) begin
            w_data <= w_data + 1;
            if (w_last) begin
                w_data <= 'b0;
            end
        end
    end

    // Write enable
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            w_fire <= 'b0;
        end
        else if (w_fire && !w_last) begin
            w_fire <= 1'b1;
        end
        else if (nstate == WRITE) begin
            w_fire <= 1'b1;
        end
        else begin
            w_fire <= 0;
        end


    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r_fire <= 'b0;
        end
        else if (r_fire && !w_last) begin
            r_fire <= 1'b1;
        end
        else if (nstate == READ) begin
            r_fire <= 1'b1;
        end
        else begin
            r_fire <= 0;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            w_last <= 'b0;
        end
        else if (t_cnt == 1) begin
            w_last <= 'b1;
        end
        else begin
            w_last <= 0;
        end
    end

    // Global Counter
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            power_cnt <= 'b0;
        end
        else begin
            power_cnt <= power_cnt + 1;
            if (nstate == LOAD_MODE_REG) begin
                power_cnt <= 0;
            end
            if (ina) begin
                if (power_cnt == AUTO_REFRESH_3) begin
                    power_cnt <= 0;
                end
            end
        end
    end

    // Cmd counter
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            t_cnt <= 'b0;
        end
        else begin
            if (t_cnt != 0) begin
                t_cnt <= t_cnt - 1;
            end
            else begin
                t_cnt <= t_cnt;
            end
            case (nstate)
                ACTIVE : t_cnt <= 'd2;

                WRITE  : begin
                    if (Mode_reg == Burst_length_f) begin
                        t_cnt <= 'd1023; // 'd1024
                    end
                    else if (Mode_reg == Burst_length_8) begin
                        t_cnt <= 'd7;
                    end
                    else if (Mode_reg == Burst_length_4) begin
                        t_cnt <= 'd3;
                    end
                    else if (Mode_reg == Burst_length_2) begin
                        t_cnt <= 'd1;
                    end
                    else if (Mode_reg == Burst_length_1) begin
                        t_cnt <= 'd0;
                    end
                end

                READ   : begin
                    if (Mode_reg == Burst_length_f) begin
                        t_cnt <= 'd1023; // 'd1024
                    end
                    else if (Mode_reg == Burst_length_8) begin
                        t_cnt <= 'd8;
                    end
                    else if (Mode_reg == Burst_length_4) begin
                        t_cnt <= 'd4;
                    end
                    else if (Mode_reg == Burst_length_2) begin
                        t_cnt <= 'd2;
                    end
                    else if (Mode_reg == Burst_length_1) begin
                        t_cnt <= 'd1;
                    end
                end

                PRECHARGE : t_cnt <= 'd2;

                LOAD_MODE_REG : t_cnt <= 'd1;

                TERMINATE : t_cnt <= 'd1;

                // default : t_cnt <= 'b0;
            endcase
        end
    end

    // Commond
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cmd_ready <= 'b0;
        end
        else if (ina) begin
            if (!t_cnt) begin
                cmd_ready <= 'b1;
            end
            else begin
                cmd_ready <= 'b0;
            end

            if (w_fire) begin
                if (ter_cnt == w_cmd_len[9 : 0] - 2) begin
                    cmd_ready <= 'b1;
                end
            end
            else if (r_fire) begin
                if (ter_cnt == r_cmd_len[9 : 0]) begin
                    cmd_ready <= 'b1;
                end
            end
        end

        if (cmd_fire) begin
            cmd_ready <= 'b0;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cmd_valid <= 0;
        end
        else if (cmd_valid && !r_complete) begin
            cmd_valid <= 1;
        end
        else if (axi_rd_fire || axi_wr_fire) begin
            cmd_valid <= 1;
        end
        else begin
            cmd_valid <= 0;
        end
    end

    // 设ter_cnt,当rw_fire拉高时计数

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ter_cnt <= 0;
        end
        else if (w_fire || r_fire) begin
            ter_cnt <= ter_cnt + 1;
            if (cmd_fire) begin
                ter_cnt <= 0;
            end
        end
    end
    

    // FSM
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cstate <= IDLE;
        end
        else begin
            cstate <= nstate;
        end
    end

    always @(*) begin
        case (cstate)
            IDLE : begin
                if (power_cnt == 'd10005) begin
                    nstate = NOP;
                end
            end
            NOP   : begin
                case (power_cnt)
                    PRECHARGE_1 : nstate = PRECHARGE;

                    AUTO_REFRESH_1 : nstate = AUTO_REFRESH;

                    AUTO_REFRESH_2 : nstate = AUTO_REFRESH;

                    AUTO_REFRESH_3 : nstate = AUTO_REFRESH;

                    LOAD_MODE_REG_1 : nstate = LOAD_MODE_REG;

                    default : nstate = NOP;
                endcase

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
        case (cstate)
            IDLE : begin
                Cs_n = 1;
            end

            NOP  : begin
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
                We_n  = 0;
            end

            default : begin
                Cs_n  = 0;
                Ras_n = 1;
                Cas_n = 1;
                We_n  = 1;
            end
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            Addr <= 'b0;
            Ba <= 'b0;
        end
        else begin
            case (nstate)
                ACTIVE : begin
                    Addr <= row_addr;
                    Ba <= cmd_ba;
                end

                READ : begin
                    Addr <= {row_addr[10], col_addr};
                    Ba <= cmd_ba;
                end

                WRITE : begin
                    Addr <= {row_addr[10], col_addr};
                    Ba <= cmd_ba;
                end

                PRECHARGE : begin
                    Addr[10] <= row_addr[10];
                    Ba <= cmd_ba;
                end

                LOAD_MODE_REG : begin
                    Addr <= row_addr;
                end
                default : begin
                    Addr <= 'b0;
                    Ba <= 'b0;
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
