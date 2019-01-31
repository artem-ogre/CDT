#include "CDT.h"

#include <QApplication>
#include <QFileDialog>
#include <QHBoxLayout>

class CDTWidget : public QWidget
{
    Q_OBJECT

public:
    explicit CDTWidget(QWidget* parent = NULL)
        : QWidget(parent)
    {
        startTimer(40);
    }

    QSize sizeHint() const
    {
        return QSize(600, 600);
    }

private:
    void timerEvent(QTimerEvent* /*e*/)
    {
        update();
    }
};

class MainWindow : public QWidget
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = Q_NULLPTR)
        : QWidget(parent)
    {
        m_cdtWidget = new CDTWidget();
        m_dir = QDir(QDir::currentPath(), tr("*.txt"));

        QHBoxLayout* centralLayout = new QHBoxLayout;
        centralLayout->addWidget(m_cdtWidget);
        setLayout(centralLayout);

        setWindowTitle(tr("CDT Visualizer"));
        layout()->setSizeConstraint(QLayout::SetFixedSize);
    }

private:
    QDir m_dir;
    CDTWidget* m_cdtWidget;
};

int main(int argc, char* argv[])
{
    typedef CDT::Triangulation<double> Triangulation;
    typedef CDT::V2d<double> V2d;
    Triangulation cdt;
    std::vector<V2d> vertices;
    vertices.push_back(V2d::make(0, 0));
    vertices.push_back(V2d::make(1, 0));
    vertices.push_back(V2d::make(2, 1));
    vertices.push_back(V2d::make(1, 1));
    cdt.insertVertices(vertices);

    QApplication app(argc, argv);
    MainWindow window;
    window.show();
    return app.exec();
}

#include "main.moc"
