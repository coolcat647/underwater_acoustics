/* For RobotX competition
   Recorder controll part is written by CT-Hung and modified to ROS version by Sam Liu
*/

#include <alsa/asoundlib.h>
#include <iostream>
#include <iomanip>
#include <ctime>
#include <deque>
#include <vector>
#include <string>
#include <signal.h>

// For ROS
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include "robotx_msgs/HydrophoneData.h"

using namespace std;

static const string FRAME_ID = "base_link";
static const string COLOR_RED = "\e[0;31m";
static const string COLOR_GREEN = "\e[0;32m";
static const string COLOR_YELLOW = "\e[0;33m"; 
static const string COLOR_NC = "\e[0m";
static const string PCM_ERR_MSG = "rosparam \"pcm_id\" is not found, please type \"arecord -l\" to check your pcm device ID,\r\nthen assign your pcm device. e,g: \"hw:2,0\"";

class RecorderNode {
    public:
        RecorderNode();                     // Constructor
        ~RecorderNode();                    // Destructor
        void run(void);

        // ROS parameter
        ros::NodeHandle nh_;                         // Private node handler
        ros::Publisher pub_sound_;                   // Private publisher instance
        robotx_msgs::HydrophoneData sound_msg_;      // Hydrophone data message

    private:
        bool setRecorderParams(void);
        void capture(void);

        // Recorder parameter
        string DEVICE_NAME_;
        string pcm_id_;
        snd_pcm_t *pcm_handle_;
        snd_pcm_hw_params_t *pcm_params_;
        snd_pcm_uframes_t pcm_frames_;
        unsigned int pcm_sampleRate_;
        unsigned int pcm_available_channels_;
        unsigned int pcm_using_channels_;
        //unsigned int pcm_recordTime_;
        unsigned int pcm_loops_;
        int pcm_dir_;
        int pcm_period_size_;
        int pcm_bits_;
        int pcm_send_size_;
        char *pcm_period_buffer_;
};

/*                                    */
/************ Constructor *************/
/*                                    */
RecorderNode::RecorderNode() {
    int tmp;
    // Import parameter from yaml file
    if (!ros::param::get("~DEVICE_NAME", DEVICE_NAME_))
        DEVICE_NAME_ = string("Focusrite 18i8 2nd");
    if (!ros::param::get("~pcm_id", pcm_id_)){
        // pcm_id_ = string("hw:0,0");
        ROS_ERROR(PCM_ERR_MSG.c_str());
        ros::shutdown();
    }
    if (!ros::param::get("~pcm_frames", tmp))
        pcm_frames_ = 100;
    else pcm_frames_ = tmp;
    if (!ros::param::get("~pcm_sampleRate", tmp))
        pcm_sampleRate_ = 96000;
    else  pcm_sampleRate_ = tmp;
    if (!ros::param::get("~pcm_available_channels", tmp))
        pcm_available_channels_ = 10;
    else pcm_available_channels_ = tmp;
    if (!ros::param::get("~pcm_using_channels", tmp))
        pcm_using_channels_ = 10;
    else pcm_using_channels_ = tmp;
    if (!ros::param::get("~pcm_bits", pcm_bits_))
        pcm_bits_ = sizeof(int32_t) * 8;
    
    // ROS publisher
    pub_sound_ = nh_.advertise<robotx_msgs::HydrophoneData>("hydrophone_data", 10);

    // Initialize message data
    sound_msg_.data_type = "int32_t";
    sound_msg_.header.frame_id = FRAME_ID;
    sound_msg_.data_ch1.clear();
    sound_msg_.data_ch2.clear();
    sound_msg_.data_ch3.clear();
    sound_msg_.data_ch4.clear();

    // Set the parameter of recorder and check if the command succeeded
    bool enable_recorder = setRecorderParams();
    if(enable_recorder == false){
        ROS_ERROR("Set recorder params failed.");
        ros::shutdown();
    } else
        cout << COLOR_GREEN << "Set recorder params sucessfully. " << COLOR_NC << endl;
    cout << COLOR_GREEN << DEVICE_NAME_ << " is ready, wait for message subscribing." << COLOR_NC << endl;
}


/*                                    */
/************* Destructor *************/
/*                                    */
RecorderNode::~RecorderNode() {
    // Stop the sound card object
    snd_pcm_drain(pcm_handle_);
    snd_pcm_close(pcm_handle_);
    free(pcm_period_buffer_);
    cout << COLOR_YELLOW << DEVICE_NAME_ << " stop recording." << COLOR_NC << endl;
}

bool RecorderNode::setRecorderParams(void) {
    int rc;
    /* Open PCM device for recording. */
    rc = snd_pcm_open(&pcm_handle_, pcm_id_.c_str(), SND_PCM_STREAM_CAPTURE, 0);
    if (rc < 0){
        cout << "Can not open record device: " << pcm_id_ << endl;
    }else
        cout << "Open device sucessfully: " << pcm_id_ << endl;
  
    /* Allcoate a hardware params object. */ 
    snd_pcm_hw_params_alloca(&pcm_params_);

    /* Fill it in with default values. */ 
    snd_pcm_hw_params_any(pcm_handle_, pcm_params_);

    /* Set the params to device. */
    /*Interleaved mode. */
    snd_pcm_hw_params_set_access(pcm_handle_, pcm_params_, SND_PCM_ACCESS_RW_INTERLEAVED);
  
    /* Signed 32-bit little-endian format. */
    snd_pcm_hw_params_set_format(pcm_handle_, pcm_params_, SND_PCM_FORMAT_S32_LE);

    /* Set channel. */
    snd_pcm_hw_params_set_channels(pcm_handle_, pcm_params_, pcm_available_channels_);

    /* Set sample rate. */
    snd_pcm_hw_params_set_rate_near(pcm_handle_, pcm_params_, &pcm_sampleRate_, &pcm_dir_);
  
    /* Set period size to frames. */
    snd_pcm_hw_params_set_period_size_near(pcm_handle_, pcm_params_, &pcm_frames_, &pcm_dir_);

    /* Write the params to the driver. */
    rc = snd_pcm_hw_params(pcm_handle_, pcm_params_);
    if(rc < 0){
        cout << "Unable to set hw params. " << endl;
        return false;
    }
    
    
    /* Decide the period size and buffer. */
    snd_pcm_hw_params_get_period_size(pcm_params_, &pcm_frames_, &pcm_dir_);
    pcm_period_size_ = pcm_frames_ * pcm_bits_ * pcm_available_channels_ / 8; //16bits // units is byte.
    pcm_period_buffer_ = (char *) malloc(pcm_period_size_);
    return true;
}

void RecorderNode::capture(void) {
    int rc;

    rc = snd_pcm_readi(pcm_handle_, pcm_period_buffer_, pcm_frames_);
    if(rc == -EPIPE){
        cout << "Overrun occurred" << endl;
        snd_pcm_prepare(pcm_handle_);
    }else if(rc < 0){
        string error = snd_strerror(rc);
        cout << "Error from read: " << error << endl;
    }else if(rc != (int)pcm_frames_){
        cout << "Short read, read wrong frames: " << rc << endl;
    }

    for(int i = 0; i < pcm_period_size_ - pcm_available_channels_ * 4 + 1; i = i + pcm_available_channels_ * 4){
        // Append data to channel according your pcm_using_channels_
        for(int j = 0; j < pcm_using_channels_; j++){
            int sum = (pcm_period_buffer_[i+j*4]) | \
                    pcm_period_buffer_[i+1+j*4] << 8 | \
                    pcm_period_buffer_[i+2+j*4] << 16 | \
                    pcm_period_buffer_[i+3+j*4] << 24;
            switch(j){
                case 0:
                    sound_msg_.data_ch1.push_back(sum); break;
                case 1:
                    sound_msg_.data_ch2.push_back(sum); break;
                case 2:
                    sound_msg_.data_ch3.push_back(sum); break;
                case 3:
                    sound_msg_.data_ch4.push_back(sum); break;
            }
        }
    }
}

void RecorderNode::run(void) {
    // Loop
    while (ros::ok()) {
        capture();
        if(sound_msg_.data_ch1.size() >= ((int)pcm_sampleRate_ / 2)){
            sound_msg_.length = sound_msg_.data_ch1.size();
            sound_msg_.header.stamp = ros::Time::now();

            if(pub_sound_.getNumSubscribers() > 0) {
                pub_sound_.publish(sound_msg_);
                ROS_INFO("Published %d samples * %d channel data.", \
                    (int)sound_msg_.length,\
                    pcm_using_channels_);
            }

            sound_msg_.data_ch1.clear();
            sound_msg_.data_ch2.clear();
            sound_msg_.data_ch3.clear();
            sound_msg_.data_ch4.clear();
        }
    }
}

int main (int argc, char** argv) {
    ros::init(argc, argv, "acoustic_recorder_node");   
    RecorderNode node;
    node.run();
    return 0;
}
