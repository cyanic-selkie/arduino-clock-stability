#include <errno.h>  // Error number definitions
#include <fcntl.h>  // File control definitions
#include <netdb.h>  // UNIX standard function definitions
#include <stdint.h> // standard input / output functions
#include <stdio.h>  // standard input / output functions
#include <stdlib.h>
#include <string.h>  // string function definitions
#include <termios.h> // POSIX terminal control definitions
#include <time.h>
#include <unistd.h> // UNIX standard function definitions

#define NSEC_PER_SEC 1000000000

void timespec_sub(struct timespec* r, const struct timespec* a,
                  const struct timespec* b) {
    r->tv_sec = a->tv_sec - b->tv_sec;
    r->tv_nsec = a->tv_nsec - b->tv_nsec;
    if (r->tv_nsec < 0) {
        r->tv_sec--;
        r->tv_nsec += NSEC_PER_SEC;
    }
}

void timespec_add_nsec(struct timespec* r, const struct timespec* a,
                       int64_t b) {
    r->tv_sec = a->tv_sec + (b / NSEC_PER_SEC);
    r->tv_nsec = a->tv_nsec + (b % NSEC_PER_SEC);

    if (r->tv_nsec >= NSEC_PER_SEC) {
        r->tv_sec++;
        r->tv_nsec -= NSEC_PER_SEC;
    } else if (r->tv_nsec < 0) {
        r->tv_sec--;
        r->tv_nsec += NSEC_PER_SEC;
    }
}

int64_t timespec_to_nsec(const struct timespec* a) {
    return (int64_t)a->tv_sec * NSEC_PER_SEC + a->tv_nsec;
}

int initialize_serial(const char* port) {
    int USB = open(port, O_RDONLY | O_NOCTTY);

    struct termios tty;
    memset(&tty, 0, sizeof tty);

    /* Error Handling */
    if (tcgetattr(USB, &tty) != 0) {
        printf("Error %d from tcgetattr: %s\n", errno, strerror(errno));
    }

    /* Set Baud Rate */
    cfsetospeed(&tty, (speed_t)B9600);
    cfsetispeed(&tty, (speed_t)B9600);

    /* Setting other Port Stuff */
    // tty.c_cflag &= ~PARENB; // Make 8n1
    // tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    tty.c_cflag &= ~CRTSCTS;       // no flow control
    tty.c_cc[VMIN] = 1;            // read blocks by count
    tty.c_cc[VTIME] = 0;           // read doesn't care about timeout
    tty.c_cflag |= CREAD | CLOCAL; // turn on READ & ignore ctrl lines

    /* Make raw */
    cfmakeraw(&tty);

    /* Flush Port, then applies attributes */
    tcflush(USB, TCIFLUSH);
    if (tcsetattr(USB, TCSANOW, &tty) != 0) {
        printf("Error %d from tcsetattr\n", errno);
    }

    return USB;
}

typedef struct {
    uint8_t li_vn_mode; // Eight bits. li, vn, and mode.
                        // li.   Two bits.   Leap indicator.
                        // vn.   Three bits. Version number of the protocol.
                        // mode. Three bits. Client will pick mode 3 for client.

    uint8_t stratum; // Eight bits. Stratum level of the local clock.
    uint8_t poll; // Eight bits. Maximum interval between successive messages.
    uint8_t precision; // Eight bits. Precision of the local clock.

    uint32_t rootDelay; // 32 bits. Total round trip delay time.
    uint32_t
        rootDispersion; // 32 bits. Max error aloud from primary clock source.
    uint32_t refId;     // 32 bits. Reference clock identifier.

    uint32_t refTm_s; // 32 bits. Reference time-stamp seconds.
    uint32_t refTm_f; // 32 bits. Reference time-stamp fraction of a second.

    uint32_t origTm_s; // 32 bits. Originate time-stamp seconds.
    uint32_t origTm_f; // 32 bits. Originate time-stamp fraction of a second.

    uint32_t rxTm_s; // 32 bits. Received time-stamp seconds.
    uint32_t rxTm_f; // 32 bits. Received time-stamp fraction of a second.

    uint32_t txTm_s; // 32 bits and the most important field the client cares
                     // about. Transmit time-stamp seconds.
    uint32_t txTm_f; // 32 bits. Transmit time-stamp fraction of a second.

} ntp_packet; // Total: 384 bits or 48 bytes.

int ntp_socket() {
    // Create a UDP socket, convert the host-name to an IP address, set the port
    // number,
    // connect to the server, send the packet, and then read in the return
    // packet.
    struct sockaddr_in serv_addr; // Server address data structure.
    struct hostent* server;       // Server data structure.

    int sockfd =
        socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP); // Create a UDP socket.

    if (sockfd < 0)
        printf("ERROR opening socket");

    server = gethostbyname("pool.ntp.org"); // Convert URL to IP.

    if (server == NULL)
        printf("ERROR, no such host");

    // Zero out the server address structure.
    bzero((char*)&serv_addr, sizeof(serv_addr));

    serv_addr.sin_family = AF_INET;

    // Copy the server's IP address to the server address structure.
    bcopy((char*)server->h_addr, (char*)&serv_addr.sin_addr.s_addr,
          server->h_length);

    // Convert the port number integer to network big-endian style and save it
    // to the server address structure.
    serv_addr.sin_port = htons(123);

    // Call up the server using its IP address and port number.
    if (connect(sockfd, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0)
        printf("ERROR connecting");

    return sockfd;
}

ntp_packet ntp_request(int sockfd) {
    // Create and zero out the packet. All 48 bytes worth.
    ntp_packet packet = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    /*memset(&packet, 0, sizeof(ntp_packet));*/

    // Set the first byte's bits to 00,011,011 for li = 0, vn
    // = 3, and mode = 3. The rest will be left set to zero.
    *((char*)&packet + 0) = 0x1b;
    // Represents 27 in base 10 or 00011011 in base 2.

    // Send it the NTP packet it wants. If n == -1, it failed.
    int n = write(sockfd, (char*)&packet, sizeof(ntp_packet));

    if (n < 0)
        printf("ERROR writing to socket");

    // Wait and receive the packet back from the server. If n == -1, it failed.
    n = read(sockfd, (char*)&packet, sizeof(ntp_packet));

    if (n < 0)
        printf("ERROR reading from socket");

    return packet;
}

void normalize_ntp_packet(ntp_packet* packet) {
    // Extract the 32 bits that represent the time-stamp seconds (since NTP
    // epoch) from when the packet left the server. Subtract 70 years worth
    // of seconds from the seconds since 1900. This leaves the seconds since
    // the UNIX epoch of 1970.
    // (1900)------------------(1970)**************************************(Time
    // Packet Left the Server)
    unsigned long long NTP_TIMESTAMP_DELTA = 2208988800ull;

    // These two fields contain the time-stamp seconds as the packet left the
    // NTP server. The number of seconds correspond to the seconds passed since
    // 1900. ntohl() converts the bit/byte order from the network's to host's
    // "endianness".
    packet->txTm_s = ntohl(packet->txTm_s) - NTP_TIMESTAMP_DELTA;
    packet->txTm_f = ntohl(packet->txTm_f);

    packet->rxTm_s = ntohl(packet->rxTm_s) - NTP_TIMESTAMP_DELTA;
    packet->rxTm_f = ntohl(packet->rxTm_f);

    // Fraction to ns
    packet->txTm_f = (uint32_t)((long double)packet->txTm_f / 4.294967296);
    packet->rxTm_f = (uint32_t)((long double)packet->rxTm_f / 4.294967296);
}

struct timespec get_true_time(int sockfd) {
    // Get time from the NTP server
    // origTm is the timestamp just before sending the request
    // finalTm is the timestamp just after receiving the response
    struct timespec origTm, finalTm, txTm, rxTm;
    clock_gettime(CLOCK_REALTIME, &origTm);
    ntp_packet packet = ntp_request(sockfd);
    clock_gettime(CLOCK_REALTIME, &finalTm);

    // Convert timestamps to the same format as origTm and finalTm
    normalize_ntp_packet(&packet);

    txTm.tv_sec = packet.txTm_s;
    txTm.tv_nsec = packet.txTm_f;
    rxTm.tv_sec = packet.rxTm_s;
    rxTm.tv_nsec = packet.rxTm_f;

    struct timespec diffServer, diffHost, delay;

    timespec_sub(&diffServer, &txTm, &rxTm);
    timespec_sub(&diffHost, &finalTm, &origTm);
    timespec_sub(&delay, &diffHost, &diffServer);

    /*printf("%ld %ld\n", origTm.tv_sec, origTm.tv_nsec);*/
    /*printf("%ld %ld\n", rxTm.tv_sec, rxTm.tv_nsec);*/
    /*printf("%ld %ld\n", txTm.tv_sec, txTm.tv_nsec);*/
    /*printf("%ld %ld\n", finalTm.tv_sec, finalTm.tv_nsec);*/

    /*printf("%ld %ld\n", diffServer.tv_sec, diffServer.tv_nsec);*/
    /*printf("%ld %ld\n", diffHost.tv_sec, diffHost.tv_nsec);*/
    /*printf("%ld %ld\n", delay.tv_sec, delay.tv_nsec);*/

    uint64_t correction = timespec_to_nsec(&delay) / 2;

    /*printf("%lu\n", correction);*/

    struct timespec correctedTm;
    timespec_add_nsec(&correctedTm, &txTm, correction);

    /*printf("%ld %ld\n", correctedTm.tv_sec, correctedTm.tv_nsec);*/

    return correctedTm;
}

int main(int argc, char* argv[]) {
    if (argc != 2) {
        printf("This program takes exactly one argument: the serial port to "
               "listen to.");
    }

    // Connect to the socket for NTP requests
    int sockfd = ntp_socket();

    int USB = initialize_serial(argv[1]);

    uint8_t response = 0;
    while (1) {
        read(USB, &response, 1);

        struct timespec trueTm = get_true_time(sockfd);

        printf("%ld %ld\n", trueTm.tv_sec, trueTm.tv_nsec);
    }

    return 0;
}
