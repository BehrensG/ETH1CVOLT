import itertools

def select_resistors(vref : float = 7.068) -> float:

    v_in_measured = vref 

    # ==========================================
    # 2. DESIGN PARAMETERS & THE OPTIMIZED 8-VALUE ARRAY
    # ==========================================
    # Highly targeted E96 1% / 0.1% base values + high-precision trim resistors
    resist_a = [7.15E+3, 7.50E+3, 8.25E+3, 8.66E+3, 9.09E+3, 9.31E+3, 49.9E+3, 1.0E+6]
    resist_b = 20.0E+3
    v_out_target = 5.0000
    MAX_PARALLEL = 3

    # Tracking variables
    best_combination = None
    best_r_a_equiv = None
    best_v_out = 0
    min_error = float('inf')

    # ==========================================
    # 3. COMBINATORIAL SEARCH ALGORITHM
    # ==========================================
    # Check combinations of 1, 2, or 3 resistors in parallel
    for r in range(1, MAX_PARALLEL + 1):
        for combo in itertools.combinations(resist_a, r):
            
            # Parallel resistance calculation: 1 / R_eq = sum(1 / R_i)
            inverse_sum = sum(1.0 / res for res in combo)
            r_eq = 1.0 / inverse_sum
            
            # Calculate voltage divider output
            v_out = vref * (resist_b / (r_eq + resist_b))
            error = abs(v_out - v_out_target)
            
            # Track the absolute best combination
            if error < min_error:
                min_error = error
                best_combination = combo
                best_r_a_equiv = r_eq
                best_v_out = v_out

    # ==========================================
    # 4. OUTPUT THE RESULTS
    # ==========================================
    print(f" Metrology Tuning Solution for V_in = {vref:.4f} V")
    print("=" * 60)
    formatted_resistors = " || ".join([f"{x/1000:.2f} kΩ" for x in best_combination])
    print(f"Configuration   : {len(best_combination)} Resistors in Parallel")
    print(f"Resistors to Use: {formatted_resistors}")
    print(f"Equivalent R_a  : {best_r_a_equiv:.2f} Ω")
    print(f"Target Output   : {v_out_target:.4f} V")
    print(f"Actual Output   : {best_v_out:.5f} V")
    print(f"Absolute Error  : {min_error * 1e6:.1f} µV ({min_error:.5f} V)")

    # Check if target tolerance is achieved
    if min_error <= 0.0001:
        print(" SUCCESS: Error is below the 0.0001 V (100 µV) threshold!")
    else:
        print("WARNING: Consider fine-tuning the base list values for this specific chip voltage.")

print(select_resistors(6.9))