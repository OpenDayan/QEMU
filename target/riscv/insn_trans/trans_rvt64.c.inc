
static bool trans_tx64_all(DisasContext *ctx, arg_tx64_all *a)
{
    /* Translate all rvtng instructions to helper function call. */
    uint64_t extInst = (((uint64_t)(a->inst64hi) << 32)) | (((uint64_t)(a->inst64lo) << 7));
    TCGv rs1 = get_gpr(ctx, (extInst >> 15) & 0x1f, EXT_NONE);
    TCGv rs2 = get_gpr(ctx, (extInst >> 20) & 0x1f, EXT_NONE);
    TCGv rs3 = get_gpr(ctx, (extInst >> 27) & 0x1f, EXT_NONE);
    TCGv rd  = get_gpr(ctx, (extInst >>  7) & 0x1f, EXT_NONE);
    TCGv dest = dest_gpr(ctx, (extInst >>  7) & 0x1f);
    TCGv_i64 inst = tcg_constant_i64(extInst | 0x3ful);
    gen_helper_tx64_all(dest, tcg_env, rs1, rs2, rs3, rd, inst);
    gen_set_gpr(ctx, (extInst >>  7) & 0x1f, dest);
    return true;
}
