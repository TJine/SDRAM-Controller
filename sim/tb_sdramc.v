module tb_sdramc;
    localparam DATA_WD = 16;
    localparam ADDR_WD = 13;
    localparam COL_WD  = 10;

    reg                           clk;
    reg                           rst_n;

    reg  [11 : 0]                 r_cmd_len;
    reg  [11 : 0]                 w_cmd_len;
    
    reg                           cmd_valid;
    reg  [3 : 0]                  cmd;
    reg  [ADDR_WD - 1 : 0]        row_addr;
    reg  [COL_WD - 1 : 0]         col_addr;
    wire  [1 : 0]                 cmd_ba;
    wire                          cmd_ready;

    wire                          Cke;
    wire                          Cs_n;
    wire                          Ras_n;
    wire                          Cas_n;
    wire                          We_n;

    wire [DATA_WD - 1 : 0]        r_data;
    wire [1 : 0]                  Dqm;
    wire [ADDR_WD - 1 : 0]        Addr;
    wire [1 : 0]                  Ba;    

    wire cmd_fire = cmd_valid && cmd_ready; 

    reg [23 : 0]                  cmd_seq;
    reg [14 : 0]                  power_cnt;

    sdramc #(.DATA_WD(DATA_WD), .ADDR_WD(ADDR_WD), .COL_WD(COL_WD))
    sdramc (
        .clk       (clk),
        .rst_n     (rst_n),
        .cmd_valid (cmd_valid),
        .cmd       (cmd),
        .r_cmd_len (r_cmd_len),
        .w_cmd_len (w_cmd_len),
        .row_addr  (row_addr),
        .col_addr  (col_addr),
        .cmd_ba    (cmd_ba),
        .cmd_ready (cmd_ready),
        .Cke       (Cke),
        .Cs_n      (Cs_n),
        .Ras_n     (Ras_n),
        .Cas_n     (Cas_n),
        .We_n      (We_n),
        .r_data    (r_data),
        .Dqm       (Dqm),
        .Addr      (Addr),
        .Ba        (Ba)
    );

    assign cmd_ba = 0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r_cmd_len <= 0;
            w_cmd_len <= 0;
        end
        else begin
            if (cmd == 'd7) begin
                w_cmd_len <= 'hfff;
            end
            if (cmd == 'd6) begin
                r_cmd_len <= 'hfff;
            end
        end
    end
  
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            row_addr <= {2'b0, 1'b1, 10'b0};
        end
        else if (power_cnt == 'd10006) begin
            row_addr <= 13'b000_0_00_010_0_111; // 修改burst_length 
        end
        else if (power_cnt == 'd10022) begin
            row_addr <= 0;
        end
        else if (cmd_fire) begin
            if (cmd == 'd5) begin
                row_addr <= row_addr + 1;
            end
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            col_addr <= 'b0;
        end
        else if (cmd_fire) begin
            if (cmd == 'd7) begin
                col_addr <= 0;
            end
            if (cmd == 'd6) begin
                col_addr <= 0;
            end
        end
    end
  
    // Global counter
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            power_cnt <= 'b0;
        end
        else begin
            power_cnt <= power_cnt + 1;
        end
    end

    // Global clock
    initial begin
        clk = 0;
        forever #10 clk = ~clk;
    end

    // rst_n
    initial begin
        rst_n = 0;
        #50 rst_n = 1;
        #220000 $finish;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cmd_valid <= 0;
        end
        else if (!cmd_valid || cmd_ready) begin
            cmd_valid <= 1;
        end
        else if (cmd_fire) begin
            cmd_valid <=0;
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
            // cmd_seq <= {4'd5,4'd7,4'd7,4'd6,4'd6,4'd2}; // Burst_length_8 act-write2-read2-precharge
            cmd_seq <= {4'd5, 4'd7, 4'd8, 4'd6,  4'd2};
        end
        else if (cmd_fire) begin
            cmd_seq <= {cmd_seq[19 : 0], 4'd0};
        end
    end
endmodule
