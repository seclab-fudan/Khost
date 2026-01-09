#include "gdbstub.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>

#define DEBUG_GDB 0
#define PORT 1234

static khost::CPU *g_cpu = nullptr;
static khost::Board *g_board = nullptr;

// Send acknowledgment ('+' or '-')
void send_ack(int client_sock, char ack) {
    char ack_packet[2] = {ack, '\0'};
    ssize_t sent = send(client_sock, &ack_packet[0], 1, 0);
    if (sent < 0) {
        perror("ERROR | khost: Failed to send acknowledgment");
    }
}

// Calculate checksum for GDB packet
uint8_t calculate_checksum(const char *data) {
    uint8_t sum = 0;
    while (*data) {
        sum += (uint8_t)*data++;
    }
    return sum;
}

// Send a packet to GDB
void send_packet(int client_sock, const char *data) {
    char packet[4096];
    uint8_t cksum = calculate_checksum(data);

    // Format the packet: $<data>#<checksum>
    snprintf(packet, sizeof(packet), "$%s#%02x", data, cksum);

    // Send the packet
    ssize_t sent = send(client_sock, packet, strlen(packet), 0);
    if (sent < 0) {
        perror("ERROR | khost: Failed to send packet");
    }

    #if DEBUG_GDB
        printf("INFO  | khost: reply: %s\n", data);
    #endif
}

// Receive a packet from GDB
char *recv_packet(int client_sock, char *buffer, size_t buffer_size, uint8_t *received_checksum) {
    ssize_t n;
    char c;
    size_t idx = 0;
    int reading = 0;

    while ((n = recv(client_sock, &c, 1, 0)) > 0) {
        if (c == '$') {
            // Start of a packet
            reading = 1;
            idx = 0;
        } else if (reading && c == '#') {
            // End of packet, read checksum
            char checksum_str[3] = {0};
            if (recv(client_sock, &checksum_str[0], 2, 0) != 2) {
                return NULL; // Failed to read checksum
            }
            *received_checksum = (uint8_t)strtol(checksum_str, NULL, 16);
            buffer[idx] = '\0'; // Null-terminate the packet
            return buffer;
        } else if (reading) {
            if (idx < buffer_size - 1) {
                buffer[idx++] = c;
            } else {
                // Buffer overflow, invalid packet
                printf("ERROR | buffer overflow in gdbstub\n");
                return NULL;
            }
        }
    }

    printf("ERROR | connection closed\n");
    return NULL; // Connection closed or error
}

// Handle 'q' queries
bool handle_query(int client_sock, const char *packet) {
    // qSupported: Tell the remote stub about features supported by GDB, and query the stub for features it supports.
    if (strncmp(packet, "qSupported", 10) == 0) {
        send_packet(client_sock, "PacketSize=4000;hwbreak+;vContSupported+");
        return true;
    }
    
    if (strncmp(packet, "qSupported+", 11) == 0) {
        send_packet(client_sock, "");
        return true;
    }

    // qfThreadInfo: Obtain a list of all active thread IDs from the target (OS)
    if (strcmp(packet, "qfThreadInfo") == 0) {
        send_packet(client_sock, "m 1");
        return true;
    }
    if (strcmp(packet, "qsThreadInfo") == 0) {
        send_packet(client_sock, "l");
        return true;
    }

    // qC: Return the current thread ID
    if (strcmp(packet, "qC") == 0) {
        send_packet(client_sock, "QC 1");
        return true;
    }
    
    send_packet(client_sock, "");
    return true;
}

// Handle 'v' extended commands
bool handle_v_command(int client_sock, const char *packet) {
    if (strcmp(packet, "vMustReplyEmpty") == 0) {
        // According to GDB RSP, reply with empty packet
        send_packet(client_sock, "");
        return true;
    } 
    
    if (strcmp(packet, "vCont?") == 0) {
        send_packet(client_sock, "vCont;c;s");
        return true;
    }
    
    // Unknown 'v' command, respond with empty
    send_packet(client_sock, "");
    return true;
}

// Handle 'g' commands: return the general registers
bool handle_g_command(int client_sock, const char *packet) {
    char buffer[1024];
    sprintf(buffer + 8*0, "%08x", __builtin_bswap32(g_cpu->get_kvm_reg(ARMv8A_R0)));
    sprintf(buffer + 8*1, "%08x", __builtin_bswap32(g_cpu->get_kvm_reg(ARMv8A_R1)));
    sprintf(buffer + 8*2, "%08x", __builtin_bswap32(g_cpu->get_kvm_reg(ARMv8A_R2)));
    sprintf(buffer + 8*3, "%08x", __builtin_bswap32(g_cpu->get_kvm_reg(ARMv8A_R3)));
    sprintf(buffer + 8*4, "%08x", __builtin_bswap32(g_cpu->get_kvm_reg(ARMv8A_R4)));
    sprintf(buffer + 8*5, "%08x", __builtin_bswap32(g_cpu->get_kvm_reg(ARMv8A_R5)));
    sprintf(buffer + 8*6, "%08x", __builtin_bswap32(g_cpu->get_kvm_reg(ARMv8A_R6)));
    sprintf(buffer + 8*7, "%08x", __builtin_bswap32(g_cpu->get_kvm_reg(ARMv8A_R7)));
    sprintf(buffer + 8*8, "%08x", __builtin_bswap32(g_cpu->get_kvm_reg(ARMv8A_R8_usr)));
    sprintf(buffer + 8*9, "%08x", __builtin_bswap32(g_cpu->get_kvm_reg(ARMv8A_R9_usr)));
    sprintf(buffer + 8*10, "%08x", __builtin_bswap32(g_cpu->get_kvm_reg(ARMv8A_R10_usr)));
    sprintf(buffer + 8*11, "%08x", __builtin_bswap32(g_cpu->get_kvm_reg(ARMv8A_R11_usr)));
    sprintf(buffer + 8*12, "%08x", __builtin_bswap32(g_cpu->get_kvm_reg(ARMv8A_R12_usr)));
    sprintf(buffer + 8*13, "%08x", __builtin_bswap32(g_cpu->get_kvm_reg(ARMv8A_SP_usr)));
    uint32_t lr = g_cpu->get_kvm_reg(ARMv8A_LR_usr);
    lr = khost::Config::instance().reloc_pc(lr & (~0b1)) | (lr & 0b1);
    sprintf(buffer + 8*14, "%08x", __builtin_bswap32(lr));
    uint32_t pc = g_cpu->get_kvm_reg(ARMv8A_PC);
    pc = khost::Config::instance().reloc_pc(pc & (~0b1)) | (pc & 0b1);
    sprintf(buffer + 8*15, "%08x", __builtin_bswap32(pc));
    for (int i = 0; i < 8; i++) {
        sprintf(buffer + 8*16 + i*24, "000000000000000000000000");
    }
    sprintf(buffer + 8*16 + 8*24, "%08x", __builtin_bswap32(0x0));
    sprintf(buffer + 8*17 + 8*24, "%08x", __builtin_bswap32(g_cpu->xpsr()));
    send_packet(client_sock, buffer);
    return true;
}

// Handle 'c' commands: continue the execution
bool handle_c_command(int client_sock, const char *packet) {
    int ret = g_board->run();
    if (ret == EXIT_MODEL) {
        khost::BoardFull *board_full = dynamic_cast<khost::BoardFull *>(g_board);
        if (board_full) {
            uint32_t pc = board_full->cpu()->get_kvm_reg(ARMv8A_PC);
            if (pc >= RUNTIME_START && pc <= RUNTIME_END) {
                uint32_t *regs = (uint32_t *)board_full->gpa_to_hva(RT_FULL_firmware_context);
                board_full->cpu()->set_kvm_reg(ARMv8A_R0, regs[0]);
                board_full->cpu()->set_kvm_reg(ARMv8A_R1, regs[1]);
                board_full->cpu()->set_kvm_reg(ARMv8A_R2, regs[2]);
                board_full->cpu()->set_kvm_reg(ARMv8A_R3, regs[3]);
                board_full->cpu()->set_kvm_reg(ARMv8A_R4, regs[4]);
                board_full->cpu()->set_kvm_reg(ARMv8A_R5, regs[5]);
                board_full->cpu()->set_kvm_reg(ARMv8A_R6, regs[6]);
                board_full->cpu()->set_kvm_reg(ARMv8A_R7, regs[7]);
                board_full->cpu()->set_kvm_reg(ARMv8A_R8_usr, regs[8]);
                board_full->cpu()->set_kvm_reg(ARMv8A_R9_usr, regs[9]);
                board_full->cpu()->set_kvm_reg(ARMv8A_R10_usr, regs[10]);
                board_full->cpu()->set_kvm_reg(ARMv8A_R11_usr, regs[11]);
                board_full->cpu()->set_kvm_reg(ARMv8A_R12_usr, regs[12]);
                board_full->cpu()->set_kvm_reg(ARMv8A_SP_usr, regs[13]);
                board_full->cpu()->set_kvm_reg(ARMv8A_LR_usr, regs[14]);
                board_full->cpu()->set_kvm_reg(ARMv8A_PC, regs[15]);
            }
        }
    }

    char buffer[1024];
    if (ret == EXIT_DEBUG_BREAKPOINT) {
        sprintf(buffer, "S05");
        send_packet(client_sock, buffer);
        return true;
    }
    if (ret == EXIT_DEBUG_WATCHPOINT) {
        int type = g_cpu->hit_hw_watchpoint().details.flags;
        switch (type)
        {
        case BP_MEM_READ:
            sprintf(buffer, "T05rwatch:%08x;thread:1;", g_cpu->hit_hw_watchpoint().details.vaddr);
            break;
        case BP_MEM_WRITE:
            sprintf(buffer, "T05watch:%08x;thread:1;", g_cpu->hit_hw_watchpoint().details.vaddr);
            break;
        case BP_MEM_ACCESS:
            sprintf(buffer, "T05awatch:%08x;thread:1;", g_cpu->hit_hw_watchpoint().details.vaddr);
            break;
        default:
            break;
        }
        send_packet(client_sock, buffer);
        return true;
    }
    if (ret == EXIT_DEBUG_SINGLESTEP) {
        send_packet(client_sock, "S05");
        return true;
    }
    if (ret == EXIT_OK || ret == EXIT_MODEL) {
        printf("INFO  | khost: %s\n", ret == EXIT_OK ? "EXIT_OK" : "EXIT_MODEL");
        send_packet(client_sock, "S09");
        return true;
    }
    send_packet(client_sock, "S09");
    return true;
}

bool handle_H_command(int client_sock, const char *packer) {
    send_packet(client_sock, "OK");
    return true;
}

bool handle_s_command(int client_sock, const char *packet) {
    g_cpu->set_single_stepping(true);
    int ret = g_board->run();
    if (ret == EXIT_MODEL) {
        khost::BoardFull *board_full = dynamic_cast<khost::BoardFull *>(g_board);
        if (board_full) {
            uint32_t pc = board_full->cpu()->get_kvm_reg(ARMv8A_PC);
            if (pc >= RUNTIME_START && pc <= RUNTIME_END) {
                uint32_t *regs = (uint32_t *)board_full->gpa_to_hva(RT_FULL_firmware_context);
                board_full->cpu()->set_kvm_reg(ARMv8A_R0, regs[0]);
                board_full->cpu()->set_kvm_reg(ARMv8A_R1, regs[1]);
                board_full->cpu()->set_kvm_reg(ARMv8A_R2, regs[2]);
                board_full->cpu()->set_kvm_reg(ARMv8A_R3, regs[3]);
                board_full->cpu()->set_kvm_reg(ARMv8A_R4, regs[4]);
                board_full->cpu()->set_kvm_reg(ARMv8A_R5, regs[5]);
                board_full->cpu()->set_kvm_reg(ARMv8A_R6, regs[6]);
                board_full->cpu()->set_kvm_reg(ARMv8A_R7, regs[7]);
                board_full->cpu()->set_kvm_reg(ARMv8A_R8_usr, regs[8]);
                board_full->cpu()->set_kvm_reg(ARMv8A_R9_usr, regs[9]);
                board_full->cpu()->set_kvm_reg(ARMv8A_R10_usr, regs[10]);
                board_full->cpu()->set_kvm_reg(ARMv8A_R11_usr, regs[11]);
                board_full->cpu()->set_kvm_reg(ARMv8A_R12_usr, regs[12]);
                board_full->cpu()->set_kvm_reg(ARMv8A_SP_usr, regs[13]);
                board_full->cpu()->set_kvm_reg(ARMv8A_LR_usr, regs[14]);
                board_full->cpu()->set_kvm_reg(ARMv8A_PC, regs[15]);
            }
        }
    }
    g_cpu->set_single_stepping(false);
    if (ret == EXIT_DEBUG_SINGLESTEP) {
        send_packet(client_sock, "S05");
    } else {
        send_packet(client_sock, "S09");
        return false;
    }
    return true;
}

// Handle 'm' commands: read memory: m<addr>,<length>
bool handle_m_command(int client_sock, const char *packet) {
    unsigned long addr, length;
    if (sscanf(packet + 1, "%lx,%lx", &addr, &length) != 2) {
        send_packet(client_sock, "E00"); // Invalid format
        return false;
    }

    uint32_t remain;
    uint8_t *raw_ptr = (uint8_t *)g_board->gpa_to_hva(addr, &remain);
    if (raw_ptr == nullptr || length > remain) {
        send_packet(client_sock, "E01"); // Error: Out of bounds
        return true;
    }

    uint8_t *ptr = new uint8_t[length];
    memcpy(ptr, raw_ptr, length);
    if (addr >= SRAM_START && addr <= SRAM_END) {
        for (int i = 0; i + 4 <= length; i++) {
            uint32_t *v = (uint32_t *)(ptr + i);
            if (*v >= ROM_OR_FLASH_START && *v <= ROM_OR_FLASH_END) {
                uint32_t rv = khost::Config::instance().reloc_pc((*v) & (~0b1));
                if (rv != *v) {
                    *v = (rv | ((*v) & 0b1));
                    i += 0x3;
                }
            }
        }
    }

    // Convert memory to hex string
    char *mem_data = new char[length * 2 + 1];
    for (unsigned long i = 0; i < length; i++) {
        sprintf(mem_data + i * 2, "%02x", ptr[i]);
    }
    mem_data[length * 2] = '\0'; // Null-terminate

    send_packet(client_sock, mem_data);
    delete [] ptr;
    delete [] mem_data;
    return true;
}

// Handle 'M' commands: write memory: M<addr>,<length>:<data>
bool handle_M_command(int client_sock, const char *packet) {
    unsigned long addr, length;
    char data[4096]; // Buffer for the data

    // Split the packet into address,length and data
    char *colon = strchr((char *)packet, ':');
    if (!colon) {
        send_packet(client_sock, "E00"); // Invalid format
        return false;
    }

    // Parse address and length
    if (sscanf(packet + 1, "%lx,%lx", &addr, &length) != 2) {
        send_packet(client_sock, "E00"); // Invalid format
        return false;
    }

    // Extract data
    strcpy(data, colon + 1);

    // Validate data length
    if (strlen(data) != length * 2) {
        send_packet(client_sock, "E00"); // Data length mismatch
        return false;
    }

    uint32_t remain;
    uint8_t *ptr = (uint8_t *)g_board->gpa_to_hva(addr, &remain);
    if (ptr == nullptr || length > remain) {
        send_packet(client_sock, "E01"); // Error: Out of bounds
        return true;
    }

    // Convert hex data to binary and write to memory
    for (unsigned long i = 0; i < length; i++) {
        unsigned int byte;
        if (sscanf(&data[i * 2], "%2x", &byte) != 1) {
            send_packet(client_sock, "E00"); // Invalid hex
            return false;
        }
        g_board->write_u8(addr + i, byte);
    }

    // Send OK if successful
    send_packet(client_sock, "OK");
    return true;
}

bool handle_Z_command(int client_sock, const char *packet) {
    uint32_t type, addr, len;
    if (sscanf(packet + 1, "%x,%x,%x", &type, &addr, &len) != 3) {
        send_packet(client_sock, "E00"); // Invalid format
        return false;
    }

    bool ok = false;
    switch (type) {
    case GDB_BREAKPOINT_SW:
    case GDB_BREAKPOINT_HW:
        ok = g_cpu->insert_hw_breakpoints(addr);
        break;
    case GDB_WATCHPOINT_READ:
    case GDB_WATCHPOINT_WRITE:
    case GDB_WATCHPOINT_ACCESS:
        ok = g_cpu->insert_hw_watchpoints(addr, len, type);
        break;
    default:
        break;
    }

    if (ok) {
        send_packet(client_sock, "OK");
        return true;
    }

    send_packet(client_sock, "");
    return false;
}

bool handle_z_command(int client_sock, const char *packet) {
    uint32_t type, addr, len;
    if (sscanf(packet + 1, "%x,%x,%x", &type, &addr, &len) != 3) {
        send_packet(client_sock, "E00"); // Invalid format
        return false;
    }

    bool ok = false;
    switch (type) {
    case GDB_BREAKPOINT_SW:
    case GDB_BREAKPOINT_HW:
        ok = g_cpu->delete_hw_breakpoints(addr);
        break;
    case GDB_WATCHPOINT_READ:
    case GDB_WATCHPOINT_WRITE:
    case GDB_WATCHPOINT_ACCESS:
        ok = g_cpu->delete_hw_watchpoints(addr, len, type);
        break;
    default:
        break;
    }

    if (ok) {
        send_packet(client_sock, "OK");
        return true;
    }

    send_packet(client_sock, "");
    return false;
}

// Function to handle GDB commands
bool handle_gdb_command(int client_sock, const char *packet) {
    if (strcmp(packet, "g") == 0) {
        return handle_g_command(client_sock, packet);
    } else if (strcmp(packet, "c") == 0) {
        return handle_c_command(client_sock, packet);
    } else if (strcmp(packet, "s") == 0) {
        return handle_s_command(client_sock, packet);
    } else if (packet[0] == 'm') {
        return handle_m_command(client_sock, packet);
    } else if (packet[0] == 'M') {
        return handle_M_command(client_sock, packet);
    } else if (packet[0] == 'z') {
        return handle_z_command(client_sock, packet);
    } else if (packet[0] == 'Z') {
        return handle_Z_command(client_sock, packet);
    } else if (packet[0] == 'H') {
        return handle_H_command(client_sock, packet);
    } else if (strcmp(packet, "?") == 0) {
        // Signal reporting (dummy response)
        send_packet(client_sock, "S05");  // Example: Signal 5 (SIGTRAP)
        return true;
    } else if (strcmp(packet, "k") == 0) {
        send_packet(client_sock, "OK");
        return true;
    } else if (packet[0] == 'q') {
        // Handle queries
        return handle_query(client_sock, packet);
    } else if (packet[0] == 'v') {
        // Handle extended commands
        return handle_v_command(client_sock, packet);
    } else {
        // Unknown command, send empty response
        send_packet(client_sock, "");
        return true;
    }
}

// Handle communication with the GDB client
void handle_client(int client_sock) {
    char packet_buffer[4096];
    uint8_t received_checksum;

    while (1) {
        // Receive packet from GDB
        char *packet = recv_packet(client_sock, packet_buffer, sizeof(packet_buffer), &received_checksum);
        if (!packet) {
            printf("ERROR | khost: Connection closed or packet error.\n");
            break;
        }

        // Calculate checksum
        uint8_t calculated_checksum = calculate_checksum(packet);
        if (calculated_checksum != received_checksum) {
            printf("INFO  | khost: Checksum mismatch: calculated %02x, received %02x\n", calculated_checksum, received_checksum);
            send_ack(client_sock, '-'); // Negative acknowledgment
            continue;
        } else {
            send_ack(client_sock, '+'); // Positive acknowledgment
        }

        #if DEBUG_GDB
            printf("INFO  | khost: gdb command: %s\n", packet);
        #endif
        
        // Handle GDB commands
        if (!handle_gdb_command(client_sock, packet)) {
            printf("INFO  | khost: Received packet from GDB client: %s\n", packet);
            return;
        }
    }
}

void debug_run(khost::Board *board) {
    int server_sock, client_sock;
    struct sockaddr_in server_addr, client_addr;
    socklen_t addr_len = sizeof(client_addr);
    char buffer[256]; // Buffer to store incoming data

    g_board = board;
    khost::BoardFull *board_full = dynamic_cast<khost::BoardFull *>(board);
    if (board_full) {
        g_cpu = board_full->cpu();
    } else {
        g_cpu = dynamic_cast<khost::BoardPara *>(board)->cpu();
    }
    g_cpu->insert_hw_breakpoints(khost::Config::instance().firmware_info()->initial_pc & (~0b1));
    g_board->run();
    g_cpu->delete_hw_breakpoints(khost::Config::instance().firmware_info()->initial_pc & (~0b1));

    // Create a socket
    if ((server_sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        perror("ERROR | khost: Socket creation failed");
        exit(EXIT_FAILURE);
    }

    // Set up the server address
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(PORT);

    // Bind the socket
    if (bind(server_sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        perror("ERROR | khost: Bind failed");
        close(server_sock);
        exit(EXIT_FAILURE);
    }

    // Listen for incoming connections
    if (listen(server_sock, 1) < 0) {
        perror("ERROR | khost: Listen failed");
        close(server_sock);
        exit(EXIT_FAILURE);
    }

    printf("INFO  | khost: GDB Stub listening on port %d...\n", PORT);

    // Accept a client connection
    if ((client_sock = accept(server_sock, (struct sockaddr *)&client_addr, &addr_len)) < 0) {
        perror("ERROR | khost: Accept failed");
        close(server_sock);
        exit(EXIT_FAILURE);
    }

    printf("INFO  | khost: GDB connected from %s\n", inet_ntoa(client_addr.sin_addr));

    // Receive data from GDB
    ssize_t n = recv(client_sock, buffer, sizeof(buffer) - 1, 0);
    if (n < 0) {
        perror("ERROR | khost: Receive failed");
    } else {
        buffer[n] = '\0'; // Null-terminate the received string
       // Receive data from GDB client
        handle_client(client_sock);
    }

    close(client_sock); // Close client socket
    close(server_sock); // Close server socket
    return;
}