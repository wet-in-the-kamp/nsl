library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library nsl_line_coding, nsl_memory, work;
use nsl_line_coding.ibm_8b10b.all;
use nsl_memory.fifo.all;
use work.flit.all;
use work.sgmii.all;

entity sgmii_elastic_buff is
  generic(
    fifo_depth_c : positive := 2048
    );
  port (
    clk_sys_i : in std_ulogic;
    clk_rx_i  : in std_ulogic;

    reset_n_i : in std_ulogic;

    symbol_i : in  nsl_line_coding.ibm_8b10b.data_t;
    symbol_o : out nsl_line_coding.ibm_8b10b.data_t;
    symbol_expected_o : out std_ulogic
    );

end entity;

architecture beh of sgmii_elastic_buff is

  -- Reg
  type state_write_t is (
    ST_IDLE,
    ST_WRITE,
    ST_SKIP,
    ST_FILL_K,
    ST_FILL_D
    );

  type state_read_t is (
    ST_IDLE,
    ST_READ,
    ST_INSERT
    );  

  type regs_write_t is
  record
    state      : state_write_t;
  end record;  
  
  type regs_read_t is
  record
    state      : state_read_t;
  end record;

  signal r_write, rin_write: regs_write_t;
  signal r_read, rin_read: regs_read_t;    

  -- Constants
  constant thresh_c : integer := fifo_depth_c / 20; -- 5% threshold  
  constant half_full_c : integer := fifo_depth_c / 2;
  
  -- Signals
  signal idle_match_write_s : std_ulogic;
  signal idle_match_read_s : std_ulogic;

  signal symbol_fifo_write_s : nsl_line_coding.ibm_8b10b.data_t;
  signal symbol_fifo_read_s : nsl_line_coding.ibm_8b10b.data_t;
  signal slv_fifo_write_s : std_ulogic_vector(8 downto 0);
  signal slv_fifo_read_s : std_ulogic_vector(8 downto 0);

  signal clock_vector_s : std_ulogic_vector(1 downto 0);
  signal fifo_out_ready_s : std_ulogic;
  signal fifo_out_valid_s : std_ulogic;
  signal fifo_in_ready_s : std_ulogic;
  signal fifo_in_valid_s : std_ulogic;  
  signal fifo_out_available_s : integer range 0 to fifo_depth_c;
  signal fifo_in_free_s : integer range 0 to fifo_depth_c;
  signal fifo_filled_s : std_ulogic;

begin  -- architecture beh

  regs_write: process(reset_n_i, clk_rx_i)
  begin
    if rising_edge(clk_rx_i) then
      r_write <= rin_write;
    end if;

    if reset_n_i = '0' then
      r_write.state <= ST_IDLE;
    end if;
  end process;

  regs_read: process(reset_n_i, clk_sys_i)
  begin
    if rising_edge(clk_sys_i) then
      r_read <= rin_read;
    end if;

    if reset_n_i = '0' then
      r_read.state <= ST_IDLE;
    end if;
  end process;

  transition_write: process(fifo_in_free_s, fifo_in_ready_s,
                            idle_match_write_s, r_write, symbol_i)
  begin
    rin_write <= r_write;

    fifo_in_valid_s <= '0';
    symbol_fifo_write_s <= K28_5;
    fifo_filled_s <= '1';

    case r_write.state is
      when ST_IDLE =>
        fifo_filled_s <= '0';
        if fifo_in_ready_s = '1' then
          rin_write.state <= ST_FILL_K;
        end if;

      when ST_FILL_K =>
        fifo_filled_s <= '0';
        symbol_fifo_write_s <= K28_5;
        fifo_in_valid_s <= '1';
        rin_write.state <= ST_FILL_D;

      when ST_FILL_D =>
        fifo_filled_s <= '0';        
        symbol_fifo_write_s <= data(16, 2);
        fifo_in_valid_s <= '1';
        if fifo_in_free_s > half_full_c then
          rin_write.state <= ST_FILL_K;
        else
          rin_write.state <= ST_WRITE;
        end if;

      when ST_WRITE =>
        if (fifo_in_free_s < thresh_c) and (idle_match_write_s = '1') then
          rin_write.state <= ST_SKIP;
        else
          symbol_fifo_write_s <= symbol_i;
          fifo_in_valid_s <= '1';
          rin_write.state <= ST_WRITE;
        end if;

      when ST_SKIP =>
        rin_write.state <= ST_WRITE;

    end case;
  end process;

  transition_read: process(fifo_filled_s, fifo_out_available_s,
                           idle_match_read_s, r_read, symbol_fifo_read_s)
  begin
    rin_read <= r_read;

    fifo_out_ready_s <= '0';
    symbol_o <= K28_5;

    case r_read.state is
      when ST_IDLE =>
        if fifo_filled_s = '1' then
          rin_read.state <= ST_READ;
        end if;

      when ST_READ =>
        if (fifo_out_available_s < thresh_c) and (idle_match_read_s = '1') then
          symbol_o <= K28_5;
          rin_read.state <= ST_INSERT;          
        else
          symbol_o <= symbol_fifo_read_s;
          fifo_out_ready_s <= '1';
          rin_read.state <= ST_READ;
        end if;

      when ST_INSERT =>
        symbol_o <= data(16,2);
        rin_read.state <= ST_READ;

    end case;
  end process;    

  -- Decode side
  sgmii_state_tracker_1: work.sgmii.sgmii_state_tracker
    port map (
      clock_i           => clk_rx_i,
      reset_n_i         => reset_n_i,
      symbol_i          => symbol_i,
      symbol_expected_o => symbol_expected_o,
      idle_match_o      => idle_match_write_s
      );

  -- FIFO
  clock_vector_s <= clk_rx_i & clk_sys_i;
  slv_fifo_write_s <= symbol_fifo_write_s.control & std_ulogic_vector(symbol_fifo_write_s.data);
  symbol_fifo_read_s.control <= slv_fifo_read_s(8);
  symbol_fifo_read_s.data <= slv_fifo_read_s(7 downto 0);
  
  fifo_homogeneous_1: nsl_memory.fifo.fifo_homogeneous
    generic map (
      data_width_c        => 9,    -- Byte + control
      word_count_c        => fifo_depth_c,
      clock_count_c       => 2
      )
    port map (
      reset_n_i           => reset_n_i,
      clock_i             => clock_vector_s,
      out_data_o          => slv_fifo_read_s,
      out_ready_i         => fifo_out_ready_s,
      out_valid_o         => fifo_out_valid_s,
      out_available_min_o => fifo_out_available_s,
      in_data_i           => slv_fifo_write_s,
      in_valid_i          => fifo_in_valid_s,
      in_ready_o          => fifo_in_ready_s,
      in_free_o           => fifo_in_free_s
      );

  -- PCS RX side
  sgmii_state_tracker_2: work.sgmii.sgmii_state_tracker
    port map (
      clock_i           => clk_sys_i,
      reset_n_i         => reset_n_i,
      symbol_i          => symbol_fifo_read_s,
      idle_match_o      => idle_match_read_s
      );
  
end architecture beh;

