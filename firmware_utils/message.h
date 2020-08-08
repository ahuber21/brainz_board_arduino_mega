#ifndef MESSAGE_H
#define MESSAGE_H

#define I2C_MSGLEN 64

void msg_put(float);
void msg_put(int);
void msg_put(const char);
void msg_put(const char*);

struct m {
  bool ready;
  int len;
  int idx;
  char text[I2C_MSGLEN];
} message;

void msg_reset() {
  message.ready = false;
  message.len = 0;
  message.idx = 0;
  memset(message.text, 0, I2C_MSGLEN);
}

void msg_put(float num) {
  char buf[10];
  memset(buf, 0, 10);
  long int decimals = 100;
  long int conv = decimals * num;
  sprintf(buf, "%ld.%ld", conv / decimals, num < 0 ? -1 * conv % decimals : conv % decimals);
  msg_put(buf);
}

void msg_put(int num) {
  char buf[4];
  memset(buf, 0, 4);
  sprintf(buf, "%d", num);
  msg_put(buf);
}

void msg_put(const char msg) { message.text[message.len++] = msg; }

void msg_put(const char* msg) {
  while (*msg) {
    message.text[message.len++] = *msg++;
  }
}

#endif
