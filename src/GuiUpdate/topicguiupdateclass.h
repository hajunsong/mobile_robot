#ifndef TOPICGUIUPDATECLASS_H
#define TOPICGUIUPDATECLASS_H

#include <QDebug>

class TopicGuiUpdateClass
{
public:
    TopicGuiUpdateClass();

    void Initialize(void* _ui, void* _model);

    void UIUpdate(void* _ui, void* _model, void* _RosClass);

private:
    int uicntTopic;
    int prevcntTopic;
    int syshzTopic;
};

#endif // TOPICGUIUPDATECLASS_H
