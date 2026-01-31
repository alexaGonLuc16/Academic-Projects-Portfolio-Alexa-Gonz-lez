# AES Encryption on FPGA (Intel)
**Duration:** April – June 2024

## Overview
This project focuses on the hardware implementation of the Advanced Encryption Standard (AES) algorithm on an Intel FPGA platform using VHDL. The main objective was to design an efficient, modular, and high-performance encryption architecture suitable for resource-constrained and real-time systems.

The project was developed as part of an academic initiative and formally presented to Intel.

## Project Objectives
- Implement the AES encryption algorithm in hardware using VHDL
- Design a modular architecture for AES rounds and key expansion
- Optimize performance and hardware resource utilization
- Validate the design through simulation.

## Technical Approach
The AES algorithm was implemented following a structured hardware design methodology. The architecture was divided into functional modules corresponding to the main AES operations, enabling scalability and efficient data flow.

## 🔐 Algorithm Reference

This implementation follows the **AES (Rijndael) cipher**, the algorithm standardized as the Advanced Encryption Standard (AES).  
The AES algorithm is a symmetric block cipher that operates on 128-bit data blocks and applies a series of transformations organized into multiple rounds.
It consists of:
- key expansion
- byte substitution (SubBytes)
- row shifting (ShiftRows)
- column mixing (MixColumns)
- key addition (AddRoundKey)

all coordinated through a control unit across multiple encryption rounds.

For a visual and conceptual reference of the encryption process and its internal transformations, see: [*AES Rijndael Cipher – Animation (ZIP)*](AES_Algorithm_Animation)<br>
<br>
## Project Implementation
A detailed explanation of the system architecture, design decisions, and experimental results is available in the presentation below:

[📄 **AES FPGA Design – Technical Presentation*](presentation/AES_FPGA_Intel_Presentation.pdf)



