#include <QApplication>
#include <QLabel>
#include <QMainWindow>

int main_d(int argc, char* argv[]) {
    QApplication app(argc, argv);

    // Create a main window
    QMainWindow window;

    // Create a label with "Hello World" text
    QLabel* label = new QLabel("Hello World");

    // Set the font to be larger
    QFont font = label->font();
    font.setPointSize(20);
    label->setFont(font);

    // Set the label as the central widget
    window.setCentralWidget(label);

    // Set the window title
    window.setWindowTitle("Qt Hello World");

    // Set a reasonable default size
    window.resize(400, 200);

    // Show the window
    window.show();

    // Run the application event loop
    return app.exec();
}