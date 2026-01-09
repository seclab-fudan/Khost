import idaapi
from idaapi import *

# While opening in IDA, set processor options to ARMv7-M for targets
# Otherwise, some instructions will be interpreted as data rather than instructions

def collect_post_call_instruction_starts():
    res = []

    # collect all heads following calls within functions
    for segea in Segments():
        for funcea in Functions(segea, get_segm_end(segea)):
            functionName = get_func_name(funcea)
            for (startea, endea) in Chunks(funcea):
                for head in Heads(startea, endea):
                    mnem = print_insn_mnem(prev_head(head, head-4))
                    if mnem and mnem.lower()[:2] == "bl":
                        res.append(head)

    return res

def collect_bbs_from_flowchart():
    seen = set()
    result = []

    for fn_addr in Functions():
        f = idaapi.FlowChart(idaapi.get_func(fn_addr))
        for block in f:
            if block.start_ea == block.end_ea:
                continue
            if block.start_ea not in seen:
                result.append((block.start_ea, block.end_ea))
            seen.add(block.start_ea)
            for succ_block in block.succs():
                if succ_block.start_ea == succ_block.end_ea:
                    continue
                if succ_block.start_ea not in seen:
                    result.append((succ_block.start_ea, succ_block.end_ea))
                seen.add(succ_block.start_ea)
            for pred_block in block.preds():
                if pred_block.start_ea == pred_block.end_ea:
                    continue
                if pred_block.start_ea not in seen:
                    result.append((pred_block.start_ea, pred_block.end_ea))
                seen.add(pred_block.start_ea)

    return result

def get_hook_coverage_points():
    result = []
    bl_ins_list = collect_post_call_instruction_starts()
 
    for bb in collect_bbs_from_flowchart():
        bb_start, bb_end = bb[0], bb[1]
        ins_in_block = set()
        ins_in_block.add(bb_start)
        for bl in bl_ins_list:
            if bb_start <= bl < bb_end:
                ins_in_block.add(bl)
        ins_in_block.add(bb_end)        
        sorted_ins_in_block = sorted(ins_in_block)
        
        for id in range(0, len(sorted_ins_in_block)-1):
            part_start = sorted_ins_in_block[id]
            part_end = sorted_ins_in_block[id+1]
            result.append((part_start, part_end))

    return result

def dump_bbl_starts_txt(out_file_path="config.yml"):
    bbs = get_hook_coverage_points()
    with open(out_file_path, "w") as f:
        f.write("basic_blocks:\n")
        for bb in sorted(bbs):
            f.write("  0x{:x}: 0x{:x}\n".format(bb[0], bb[1]))
        
        f.write("symbols:\n")
        for addr, name in Names():
            f.write("  0x{:x}: '{}'\n".format(addr, name))
            
dump_bbl_starts_txt()