#ifndef _BLUETOOTH_RECEIVER_HPP_
#define _BLUETOOTH_RECEIVER_HPP_

class BluetoothReceiver {
    private:
    public:
        BluetoothReceiver();

        bool init(void);
};

extern BluetoothReceiver bt_receiver;

#endif // _BLUETOOTH_RECEIVER_HPP_