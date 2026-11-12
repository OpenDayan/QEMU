#include "qemu/osdep.h"

#include "hw/riscv/dayan-intf.h"

static void close_pair(int sockets[2])
{
    close(sockets[0]);
    close(sockets[1]);
}

static void make_pair(int sockets[2])
{
    g_assert_cmpint(socketpair(AF_UNIX, SOCK_SEQPACKET, 0, sockets), ==, 0);
}

static void test_push_is_one_exact_record(void)
{
    const uint64_t expected = UINT64_C(0x0123456789abcdef);
    uint8_t buffer[2 * sizeof(expected)] = { 0 };
    struct iovec iov = {
        .iov_base = buffer,
        .iov_len = sizeof(buffer),
    };
    struct msghdr message = {
        .msg_iov = &iov,
        .msg_iovlen = 1,
    };
    int sockets[2];
    ssize_t received;

    make_pair(sockets);

    g_assert_cmpint(dayan_socket_push(sockets[0], expected), ==, 0);
    received = recvmsg(sockets[1], &message, 0);

    g_assert_cmpint(received, ==, sizeof(expected));
    g_assert_cmpint(message.msg_flags & MSG_TRUNC, ==, 0);
    g_assert_cmpmem(buffer, sizeof(expected), &expected, sizeof(expected));

    close_pair(sockets);
}

static void test_round_trip(void)
{
    const uint64_t expected = UINT64_C(0xfedcba9876543210);
    uint64_t actual = 0;
    int sockets[2];

    make_pair(sockets);

    g_assert_cmpint(dayan_socket_push(sockets[0], expected), ==, 0);
    g_assert_cmpint(dayan_socket_pop(sockets[1], &actual), ==, 0);
    g_assert_cmpuint(actual, ==, expected);

    close_pair(sockets);
}

static void test_pop_rejects_fragmented_record(void)
{
    const uint64_t value = UINT64_C(0x1122334455667788);
    const uint8_t *value_bytes = (const uint8_t *)&value;
    uint64_t actual = 0;
    int sockets[2];

    make_pair(sockets);

    g_assert_cmpint(send(sockets[0], &value, sizeof(value) / 2,
                         MSG_NOSIGNAL), ==, sizeof(value) / 2);
    g_assert_cmpint(send(sockets[0], value_bytes + sizeof(value) / 2,
                         sizeof(value) / 2, MSG_NOSIGNAL), ==,
                    sizeof(value) / 2);

    g_assert_cmpint(dayan_socket_pop(sockets[1], &actual), ==, -1);

    close_pair(sockets);
}

static void test_pop_rejects_oversized_record(void)
{
    uint64_t record[2] = {
        UINT64_C(0x0123456789abcdef),
        UINT64_C(0xfedcba9876543210),
    };
    uint64_t actual = 0;
    int sockets[2];

    make_pair(sockets);

    g_assert_cmpint(send(sockets[0], record, sizeof(record), MSG_NOSIGNAL), ==,
                    sizeof(record));
    g_assert_cmpint(dayan_socket_pop(sockets[1], &actual), ==, -1);

    close_pair(sockets);
}

static void assert_timeout_disabled(int socket_fd, int option)
{
    struct timeval timeout = { -1, -1 };
    socklen_t timeout_size = sizeof(timeout);

    g_assert_cmpint(getsockopt(socket_fd, SOL_SOCKET, option, &timeout,
                               &timeout_size), ==, 0);
    g_assert_cmpuint(timeout_size, ==, sizeof(timeout));
    g_assert_cmpint(timeout.tv_sec, ==, 0);
    g_assert_cmpint(timeout.tv_usec, ==, 0);
}

static void test_tune_leaves_steady_state_timeouts_disabled(void)
{
    int sockets[2];

    make_pair(sockets);
    dayan_socket_tune(sockets[0]);

    assert_timeout_disabled(sockets[0], SO_SNDTIMEO);
    assert_timeout_disabled(sockets[0], SO_RCVTIMEO);

    close_pair(sockets);
}

static void assert_interrupt_message(int socket_fd, uint64_t core_idx,
                                     uint64_t level)
{
    uint64_t actual = 0;

    g_assert_cmpint(dayan_socket_pop(socket_fd, &actual), ==, 0);
    g_assert_cmpuint(actual, ==, 0x1);
    g_assert_cmpint(dayan_socket_pop(socket_fd, &actual), ==, 0);
    g_assert_cmpuint(actual, ==, core_idx);
    g_assert_cmpint(dayan_socket_pop(socket_fd, &actual), ==, 0);
    g_assert_cmpuint(actual, ==, level);
}

static void test_interrupt_level_message(void)
{
    int sockets[2];

    make_pair(sockets);

    g_assert_cmpint(backend_interrupt_update(sockets[0], 3, 1), ==, 1);
    assert_interrupt_message(sockets[1], 3, 1);

    g_assert_cmpint(backend_interrupt_update(sockets[0], 3, 0), ==, 1);
    assert_interrupt_message(sockets[1], 3, 0);

    close_pair(sockets);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    g_test_add_func("/dayan/ipc/push-one-exact-record",
                    test_push_is_one_exact_record);
    g_test_add_func("/dayan/ipc/round-trip", test_round_trip);
    g_test_add_func("/dayan/ipc/reject-fragmented-record",
                    test_pop_rejects_fragmented_record);
    g_test_add_func("/dayan/ipc/reject-oversized-record",
                    test_pop_rejects_oversized_record);
    g_test_add_func("/dayan/ipc/no-steady-state-timeouts",
                    test_tune_leaves_steady_state_timeouts_disabled);
    g_test_add_func("/dayan/ipc/interrupt-level-message",
                    test_interrupt_level_message);

    return g_test_run();
}
