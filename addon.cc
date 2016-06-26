#include <node.h>
#include <mraa.h>
#include <iostream>
#include <vector>

using namespace v8;

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
        printf("Number of array elements: %d\n",numValues);

    for (unsigned int i = 0; i < numValues; i++) {
        //printf("Value: %d", (int)input->Get(i)->NumberValue());
    }
        args.GetReturnValue().Set(true);

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
  NODE_SET_METHOD(exports, "add", Add);
  NODE_SET_METHOD(exports, "write", Write);
}

NODE_MODULE(addon, Init);
