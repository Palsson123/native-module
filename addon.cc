#include <node.h>
#include <mraa.h>
#include <iostream>
#include <vector>
#include <stdio.h>
#include <fcntl.h>
#include <stdint.h>
#include <string.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
using namespace v8;
#define LED_PIN      44        /**< The pin where the LED is connected */


typedef struct {
    mraa_result_t (*gpio_init_internal_replace) (mraa_gpio_context dev, int pin);
    mraa_result_t (*gpio_init_pre) (int pin);
    mraa_result_t (*gpio_init_post) (mraa_gpio_context dev);

    mraa_result_t (*gpio_close_pre) (mraa_gpio_context dev);
    mraa_result_t (*gpio_close_replace) (mraa_gpio_context dev);

    mraa_result_t (*gpio_mode_replace) (mraa_gpio_context dev, mraa_gpio_mode_t mode);
    mraa_result_t (*gpio_mode_pre) (mraa_gpio_context dev, mraa_gpio_mode_t mode);
    mraa_result_t (*gpio_mode_post) (mraa_gpio_context dev, mraa_gpio_mode_t mode);

    mraa_result_t (*gpio_edge_mode_replace) (mraa_gpio_context dev, mraa_gpio_edge_t mode);

    mraa_result_t (*gpio_dir_replace) (mraa_gpio_context dev, mraa_gpio_dir_t dir);
    mraa_result_t (*gpio_dir_pre) (mraa_gpio_context dev, mraa_gpio_dir_t dir);
    mraa_result_t (*gpio_dir_post) (mraa_gpio_context dev, mraa_gpio_dir_t dir);

    int (*gpio_read_replace) (mraa_gpio_context dev);
    mraa_result_t (*gpio_write_replace) (mraa_gpio_context dev, int value);
    mraa_result_t (*gpio_write_pre) (mraa_gpio_context dev, int value);
    mraa_result_t (*gpio_write_post) (mraa_gpio_context dev, int value);
    mraa_result_t (*gpio_mmap_setup) (mraa_gpio_context dev, mraa_boolean_t en);
    mraa_result_t (*gpio_interrupt_handler_init_replace) (mraa_gpio_context dev);
    mraa_result_t (*gpio_wait_interrupt_replace) (mraa_gpio_context dev);

    mraa_result_t (*i2c_init_pre) (unsigned int bus);
    mraa_result_t (*i2c_init_bus_replace) (mraa_i2c_context dev);
    mraa_i2c_context (*i2c_init_raw_replace) (unsigned int bus);
    mraa_result_t (*i2c_init_post) (mraa_i2c_context dev);
    mraa_result_t (*i2c_set_frequency_replace) (mraa_i2c_context dev, mraa_i2c_mode_t mode);
    mraa_result_t (*i2c_address_replace) (mraa_i2c_context dev, uint8_t addr);
    int (*i2c_read_replace) (mraa_i2c_context dev, uint8_t* data, int length);
    int (*i2c_read_byte_replace) (mraa_i2c_context dev);
    int (*i2c_read_byte_data_replace) (mraa_i2c_context dev, const uint8_t command);
    int (*i2c_read_word_data_replace) (mraa_i2c_context dev, const uint8_t command);
    int (*i2c_read_bytes_data_replace) (mraa_i2c_context dev, uint8_t command, uint8_t* data, int length);
    mraa_result_t (*i2c_write_replace) (mraa_i2c_context dev, const uint8_t* data, int length);
    mraa_result_t (*i2c_write_byte_replace) (mraa_i2c_context dev, uint8_t data);
    mraa_result_t (*i2c_write_byte_data_replace) (mraa_i2c_context dev, const uint8_t data, const uint8_t command);
    mraa_result_t (*i2c_write_word_data_replace) (mraa_i2c_context dev, const uint16_t data, const uint8_t command);
    mraa_result_t (*i2c_stop_replace) (mraa_i2c_context dev);

    mraa_result_t (*aio_init_internal_replace) (mraa_aio_context dev, int pin);
    int (*aio_read_replace) (mraa_aio_context dev);
    mraa_result_t (*aio_get_valid_fp) (mraa_aio_context dev);
    mraa_result_t (*aio_init_pre) (unsigned int aio);
    mraa_result_t (*aio_init_post) (mraa_aio_context dev);

    mraa_pwm_context (*pwm_init_replace) (int pin);
    mraa_pwm_context (*pwm_init_internal_replace) (void* func_table, int pin);
    mraa_result_t (*pwm_init_pre) (int pin);
    mraa_result_t (*pwm_init_post) (mraa_pwm_context pwm);
    mraa_result_t (*pwm_period_replace) (mraa_pwm_context dev, int period);
    float (*pwm_read_replace) (mraa_pwm_context dev);
    mraa_result_t (*pwm_write_replace) (mraa_pwm_context dev, float duty);
    mraa_result_t (*pwm_write_pre) (mraa_pwm_context dev, float percentage);
    mraa_result_t (*pwm_enable_replace) (mraa_pwm_context dev, int enable);
    mraa_result_t (*pwm_enable_pre) (mraa_pwm_context dev, int enable);

    mraa_result_t (*spi_init_pre) (int bus);
    mraa_result_t (*spi_init_post) (mraa_spi_context spi);
    mraa_result_t (*spi_lsbmode_replace) (mraa_spi_context dev, mraa_boolean_t lsb);

    mraa_result_t (*uart_init_pre) (int index);
    mraa_result_t (*uart_init_post) (mraa_uart_context uart);
} mraa_adv_func_t;





struct _spi {
    /*@{*/
    int devfd;          /**< File descriptor to SPI Device */
    uint32_t mode;      /**< Spi mode see spidev.h */
    int clock;          /**< clock to run transactions at */
    mraa_boolean_t lsb; /**< least significant bit mode */
    unsigned int bpw;   /**< Bits per word */
    mraa_adv_func_t* advance_func; /**< override function table */
    /*@}*/
};

mraa_spi_context dev;
char data[32];
char counter=0;
struct spi_ioc_transfer msg;

void Write(const FunctionCallbackInfo<Value>& args) {
    Isolate* isolate = args.GetIsolate();
    // Make sure there is an argument.
    if (args.Length() != 1) {
        isolate->ThrowException(Exception::TypeError(
            String::NewFromUtf8(isolate, "Need an argument")));
        return;
    }

    // Make sure it's an array.
    if (! args[0]->IsArray()) {
        isolate->ThrowException(Exception::TypeError(
            String::NewFromUtf8(isolate, "First argument needs to be an array")));
        return;
    }

    // Unpack JS array into a std::vector
    std::vector<int> values;
    Local<Array> input = Local<Array>::Cast(args[0]);
    unsigned int numValues = input->Length();
        //printf("Number of array elements: %d\n",numValues);
    
    msg.rx_buf = 0; // Block SPI from reading anything.
    msg.len = 32;
    for (unsigned int i = 0; i < numValues; i++) {
            
            data[counter++] = (char)input->Get(i)->NumberValue();
            if(counter == 32)
            {
              if (ioctl(dev->devfd, SPI_IOC_MESSAGE(1), &msg) < 0) {
             }	
             counter=0;
            }
    }
    if(counter!=0)
    {
        msg.len = counter;   
         if (ioctl(dev->devfd, SPI_IOC_MESSAGE(1), &msg) < 0) {
         }	
    }
        args.GetReturnValue().Set(true);

}
void WriteRead(const FunctionCallbackInfo<Value>& args) {
    Isolate* isolate = args.GetIsolate();
    // Make sure there is an argument.
    if (args.Length() != 1) {
        isolate->ThrowException(Exception::TypeError(
            String::NewFromUtf8(isolate, "Need an argument")));
        return;
    }

    // Make sure it's an array.
    if (! args[0]->IsArray()) {
        isolate->ThrowException(Exception::TypeError(
            String::NewFromUtf8(isolate, "First argument needs to be an array")));
        return;
    }

    // Unpack JS array into a std::vector
    std::vector<int> values;
    Local<Array> input = Local<Array>::Cast(args[0]);
    unsigned int numValues = input->Length();
        //printf("Number of array elements: %d\n",numValues);
    char rx=0;
    msg.rx_buf = (unsigned long) &rx; // Block SPI from reading anything.
    for (unsigned int i = 0; i < numValues; i++) {
            data[0] = (char)input->Get(i)->NumberValue();
          if (ioctl(dev->devfd, SPI_IOC_MESSAGE(1), &msg) < 0) {
         }	
        //printf("Value: %d", (int)input->Get(i)->NumberValue());
    }
        args.GetReturnValue().Set(rx);

}
void Add(const FunctionCallbackInfo<Value>& args) {
  Isolate* isolate = Isolate::GetCurrent();
  HandleScope scope(isolate);
  
  if (args.Length() < 2) {
    isolate->ThrowException(Exception::TypeError(
          String::NewFromUtf8(isolate, "Wrong number of arguments")));
    return;
  }
  
  if (!args[0]->IsNumber() || !args[1]->IsNumber()) {
    isolate->ThrowException(Exception::TypeError(
          String::NewFromUtf8(isolate, "Wrong arguments")));
  }

  double value = args[0]->NumberValue() + args[1]->NumberValue();
  Local<Number> num = Number::New(isolate, value);

  args.GetReturnValue().Set(num);
}

void Init(Handle<Object> exports) {
    dev = (mraa_spi_context) calloc(1, sizeof(struct _spi));
    
    memset(&msg, 0, sizeof(msg));

    char path[64];
    sprintf(path, "/dev/spidev%u.%u", 32766,1);

    dev->devfd = open(path, O_RDWR);

    char length = 1;
    msg.tx_buf = (unsigned long) data;
    msg.rx_buf = 0;
    msg.speed_hz = 7000000;
    msg.bits_per_word = 8;
    msg.delay_usecs = 0;
    msg.len = length;
      
    
    
    
  NODE_SET_METHOD(exports, "add", Add);
  NODE_SET_METHOD(exports, "write", Write);
  NODE_SET_METHOD(exports, "writeread", WriteRead);
  
}

NODE_MODULE(addon, Init);
