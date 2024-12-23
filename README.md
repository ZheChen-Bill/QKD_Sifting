# Sifting Module

## Overview
The sifting module in the Quantum Key Distribution (QKD) system is a critical component that ensures proper key generation by filtering undetected and decoy signals. The module operates using the **Coherent One-Way (COW) protocol** \cite{COWprotocol}, where encoding a single qubit requires two pulse bits, as shown in the table below.

| **Qubit**          | **Pulse Bit** |
|---------------------|---------------|
| Qubit Decoy         | `2'b11`       |
| Qubit Not Detected  | `2'b00`       |
| Qubit 0             | `2'b10`       |
| Qubit 1             | `2'b01`       |

The module performs two main functions:
1. **Eliminating extraneous positions**, such as undetected photons and decoys, to generate the sifted key.
2. **Recording visibility parameters** for further analysis.

Before the sifting process begins, both **Alice** and **Bob** must supply the required qubit data and detected positions. The workflow is detailed below.

---

## Workflow

### Bob's Sifting Module
The sifting process for Bob is initiated by transmitting detected position data for the X-basis and Z-basis to Alice. The module then:
- Waits for a valid read signal from the decoy FIFO to eliminate unwanted positions.
- Writes accumulated sifted key data (64 bits) into the `siftedkey URAM`.
- Input/output details and the block diagram of Bob's sifting module are illustrated below.

#### Bob's Module I/O
![圖片](https://github.com/user-attachments/assets/3ebe1630-fb37-4dd4-ad7c-b2422e65e0d0)

#### Bob's Module Block Diagram
![圖片](https://github.com/user-attachments/assets/5dc9574d-af48-4274-baaa-85fc5b14d4b0)

---

### Alice's Sifting Module
Alice's module operates by:
- Continuously reading from the **Qubit BRAM** and **Z-basis/X-basis detected RX FIFOs**.
- Performing simultaneous sifting and visibility parameter calculations.
- Storing 64-bit sifted keys in the `siftedkey URAM` and transferring visibility parameters to the `visibility parameter FIFO` after accumulating 8192 bits.

#### Alice's Module I/O
![圖片](https://github.com/user-attachments/assets/76a4beac-9b9a-4fdf-a184-98974b5a20a4)

#### Alice's Module Block Diagram
![圖片](https://github.com/user-attachments/assets/dfc6f18f-75b4-46d0-a063-040be1b7da9c)

---

## Process Flow
The entire sifting process involves the following steps:
1. Bob transmits X-basis and Z-basis signals to Alice via the network layer.
2. Alice processes the data to perform sifting while simultaneously calculating visibility parameters.
3. Alice transmits the decoy positions back to Bob to proceed with further sifting.

This process is summarized in the diagram below.
![圖片](https://github.com/user-attachments/assets/3478ac23-d7e0-414a-a913-7f08e5216cda)

![圖片](https://github.com/user-attachments/assets/5fc23780-ab8b-45f4-b167-f4f62e585054)

---

## Key Features
- **Network Layer Integration**: Ensures robust and reliable communication between Alice and Bob, facilitating the exchange of qubit data and decoy positions.
- **Efficiency**: Processes 32 qubits per cycle, achieving high throughput for key distillation.
- **Preparation for Distillation**: Sets the foundation for subsequent processes like error reconciliation and privacy amplification.

The sifting module plays a vital role in ensuring the integrity and efficiency of the QKD system by filtering unnecessary data and preparing accurate sifted keys for further processing.
